#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 舵轮底盘控制实现，由 Chassis 模板使用 / Helm (swerve) chassis controller used by the Chassis template
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */
#define GM6020_NM_TO_LSB_RATIO 7370.0f /* 6020转子扭矩转化为电机控制单位的比例 */

#define HELM_CHASSIS_MAX_POWER 100

struct ChassisVelocity
{
  float x = 0.0f;      // m/s, +x right
  float y = 0.0f;      // m/s, +y forward
  float omega = 0.0f;  // rad/s, counterclockwise positive
  bool valid = false;
};

template <typename ChassisType>
class Chassis;

/**
 * @brief 舵轮底盘控制实现，由 Chassis 模板使用。
 *        Helm (swerve) chassis controller used by the Chassis template.
 */
class Helm
{
 public:
  /**
   * @brief 底盘几何、动力与小陀螺缩放参数。
   *        Chassis geometry, dynamics and spin-scaling parameters.
   */
  struct ChassisParam
  {
    float wheel_radius = 0.0f;  ///< 轮半径 (m)
    ///< Wheel radius (m)
    float wheel_to_center = 0.0f;  ///< 轮心到底盘中心的距离 (m)
    ///< Distance from the wheel center to the chassis center (m)
    float gravity_height = 0.0f;  ///< 质心高度 (m)，用于全向轮姿态前馈
    ///< Center-of-mass height (m), used by the Omni attitude feedforward
    float reduction_ratio = 0.0f;  ///< 轮电机减速比
    ///< Reduction ratio of the wheel motors
    float wheel_resistance = 0.0f;  ///< 轮阻
    ///< Wheel resistance
    float error_compensation = 0.0f;  ///< 误差补偿
    ///< Error compensation
    float gravity = 0.0f;  ///< 底盘重力 (N)，用于全向轮姿态前馈
    ///< Chassis weight (N), used by the Omni attitude feedforward
    float length = 0.0f;  ///< 轮距的长 (m)，用于全向轮姿态前馈
    ///< Length of the wheel layout (m), used by the Omni attitude feedforward
    float width = 0.0f;  ///< 轮距的宽 (m)，用于全向轮姿态前馈
    ///< Width of the wheel layout (m), used by the Omni attitude feedforward
    float rotor_speed_scale = 1.0f;  ///< 平移输入下的小陀螺转速缩放比例
    ///< Spin speed scale under translation input
    float rotor_omega_min_scale = 0.55f;  ///< 小陀螺动态缩放的下限
    ///< Lower bound of the dynamic spin scale
    float rotor_buffer_low_j = 35.0f;  ///< 缓冲能量低阈值 (J)
    ///< Low buffer energy threshold (J)
    float rotor_buffer_high_j = 70.0f;  ///< 缓冲能量高阈值 (J)
    ///< High buffer energy threshold (J)
    float rotor_scale_lpf_alpha = 0.2f;  ///< 动态缩放的一阶低通系数
    ///< First-order low-pass coefficient of the dynamic scale
    float moment_of_inertia = 0.0f;  ///< Yaw inertia (kg*m^2), zero disables feedforward
  };
  /**
   * @brief 底盘模式。
   *        Chassis modes.
   */
  enum class ChassisMode : uint8_t
  {
    RELAX,  ///< 放松：全部电机放松
    ///< Relax: all motors relaxed
    INDEPENDENT,  ///< 独立：平移相对底盘坐标系
    ///< Independent: translation in the chassis frame
    ROTOR,  ///< 小陀螺：底盘旋转，平移相对云台坐标系
    ///< Spin: the chassis rotates, translation in the gimbal frame
    FOLLOW  ///< 跟随：底盘跟随云台 yaw，平移相对云台坐标系
    ///< Follow: the chassis follows the gimbal yaw, translation in the gimbal frame
  };

  /**
   * @brief 构造舵轮底盘控制对象，创建控制线程。
   *        Construct the helm chassis controller and create the control thread.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 第 0 个舵向电机，非空。
   *                      Steering motor 0, non-null.
   * @param motor_steer_1 第 1 个舵向电机，非空。
   *                      Steering motor 1, non-null.
   * @param motor_steer_2 第 2 个舵向电机，非空。
   *                      Steering motor 2, non-null.
   * @param motor_steer_3 第 3 个舵向电机，非空。
   *                      Steering motor 3, non-null.
   * @param cmd 控制命令模块实例。
   *            Control command Module instance.
   * @param power_control 功率控制模块实例。
   *                      Power control Module instance.
   * @param referee 裁判系统模块实例。
   *                Referee Module instance.
   * @param task_stack_depth 控制线程栈深。
   *                         Control thread stack depth.
   * @param chassis_param 底盘几何与动力参数。
   *                      Chassis geometry and dynamics parameters.
   * @param pid_follow 跟随云台的角度 PID 参数。
   *                   PID parameters of the gimbal-following angle loop.
   * @param pid_velocity_x x 方向速度 PID 参数。
   *                       PID parameters of the x velocity loop.
   * @param pid_velocity_y y 方向速度 PID 参数。
   *                       PID parameters of the y velocity loop.
   * @param pid_omega 角速度 PID 参数。
   *                  PID parameters of the angular-velocity loop.
   * @param pid_wheel_speed_0 轮 0 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 0.
   * @param pid_wheel_speed_1 轮 1 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 1.
   * @param pid_wheel_speed_2 轮 2 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 2.
   * @param pid_wheel_speed_3 轮 3 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 3.
   * @param pid_steer_angle_0 舵 0 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 0.
   * @param pid_steer_angle_1 舵 1 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 1.
   * @param pid_steer_angle_2 舵 2 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 2.
   * @param pid_steer_angle_3 舵 3 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 3.
   * @param pid_steer_speed_0 舵 0 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 0.
   * @param pid_steer_speed_1 舵 1 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 1.
   * @param pid_steer_speed_2 舵 2 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 2.
   * @param pid_steer_speed_3 舵 3 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 3.
   * @param thread_priority 控制线程优先级。
   *                        Control thread priority.
   * @param topic_names 订阅的 Topic 名称。
   *                    Names of the subscribed Topics.
   */
  Helm(Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
       Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
       Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd, PowerControl* power_control,
       Referee* referee, uint32_t task_stack_depth, ChassisParam chassis_param,
       LibXR::PID<float>::Param pid_follow, LibXR::PID<float>::Param pid_velocity_x,
       LibXR::PID<float>::Param pid_velocity_y, LibXR::PID<float>::Param pid_omega,
       LibXR::PID<float>::Param pid_wheel_speed_0,
       LibXR::PID<float>::Param pid_wheel_speed_1,
       LibXR::PID<float>::Param pid_wheel_speed_2,
       LibXR::PID<float>::Param pid_wheel_speed_3,
       LibXR::PID<float>::Param pid_steer_angle_0,
       LibXR::PID<float>::Param pid_steer_angle_1,
       LibXR::PID<float>::Param pid_steer_angle_2,
       LibXR::PID<float>::Param pid_steer_angle_3,
       LibXR::PID<float>::Param pid_steer_speed_0,
       LibXR::PID<float>::Param pid_steer_speed_1,
       LibXR::PID<float>::Param pid_steer_speed_2,
       LibXR::PID<float>::Param pid_steer_speed_3,
       LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH,
       ChassisTopicNames topic_names = {})
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        motor_steer_0_(motor_steer_0),   /* wheel3   ▲ y  wheel2 */
        motor_steer_1_(motor_steer_1),   /*     ↑    │     ↑     */
        motor_steer_2_(motor_steer_2),   /*          │           */
        motor_steer_3_(motor_steer_3),   /* ─────────┼────────▶x */
        pid_follow_(pid_follow),         /*          │           */
        pid_velocity_x_(pid_velocity_x), /*     ↑    │     ↑     */
        pid_velocity_y_(pid_velocity_y), /* wheel0   │    wheel1 */
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1, pid_wheel_speed_2,
                         pid_wheel_speed_3},
        pid_steer_angle_{
            pid_steer_angle_0,
            pid_steer_angle_1,
            pid_steer_angle_2,
            pid_steer_angle_3,
        },
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1, pid_steer_speed_2,
                         pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control)
  {
    topic_names_ = topic_names;
    ASSERT(motor_steer_0 != nullptr);
    ASSERT(motor_steer_1 != nullptr);
    ASSERT(motor_steer_2 != nullptr);
    ASSERT(motor_steer_3 != nullptr);
    UNUSED(referee);

    for (int i = 0; i < 4; i++)
    {
      motor_wheel_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_wheel_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_wheel_cmd_[i].torque = 0.0f;
      motor_wheel_cmd_[i].position = 0.0f;
      motor_wheel_cmd_[i].velocity = 0.0f;
      motor_wheel_cmd_[i].kp = 0.0f;
      motor_wheel_cmd_[i].kd = 0.0f;
    }
    for (int i = 0; i < 4; i++)
    {
      motor_steer_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_steer_cmd_[i].reduction_ratio = 1.0f;
      motor_steer_cmd_[i].torque = 0.0f;
      motor_steer_cmd_[i].position = 0.0f;
      motor_steer_cmd_[i].velocity = 0.0f;
      motor_steer_cmd_[i].kp = 0.0f;
      motor_steer_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "HelmChassisThread", task_stack_depth,
                   thread_priority);
    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm* helm, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->mutex_.Lock();
          helm->chassis_event_ = ChassisMode::RELAX;
          helm->mutex_.Unlock();
        },
        this);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm* helm, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->mutex_.Lock();
          helm->chassis_event_ = ChassisMode::RELAX;
          helm->mutex_.Unlock();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 舵轮底盘控制线程函数
   * @param helm Helm对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Helm* helm)
  {
    helm->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(
        helm->topic_names_.chassis_cmd);
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        helm->topic_names_.chassis_ref);
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    helm->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    helm->mutex_.Unlock();
    while (true)
    {
      if (cmd_suber.Available())
      {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available())
      {
        helm->referee_chassis_pack_ = referee_suber.GetData();
        helm->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available())
      {
        helm->current_yaw_ = LibXR::CycleValue<float>(yawmotor_angle_suber.GetData());
        helm->delta_yaw_ = -helm->current_yaw_;
        yawmotor_angle_suber.StartWaiting();
      }

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateCMD();
      helm->topic_delta_yaw_.Publish(helm->delta_yaw_);
      helm->SelfResolution();
      helm->InverseKinematicsSolution();
      helm->OptimizeWheelAngle();
      helm->CompensateWheelSpeed();
      helm->DynamicInverseSolution();
      helm->CalculateMotorCurrent();
      helm->PowerControlUpdate();
      helm->mutex_.Unlock();
      helm->Output();
      helm->thread_.SleepUntil(last_time, 2);
    }
  }
  /**
   * @brief 更新电机状态
   * @details 获取当前时间戳并更新所有驱动轮电机的状态
   */
  void Update()
  {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    feedback_valid_ = true;
    for (int i = 0; i < 4; i++)
    {
      const auto WHEEL_STATUS = motor_wheel_[i]->Update();
      const auto STEER_STATUS = motor_steer_[i]->Update();
      motor_wheel_feedback_[i] = motor_wheel_[i]->GetFeedback();
      motor_steer_feedback_[i] = motor_steer_[i]->GetFeedback();
      feedback_valid_ = feedback_valid_ && WHEEL_STATUS == LibXR::ErrorCode::OK &&
                        STEER_STATUS == LibXR::ErrorCode::OK &&
                        motor_wheel_feedback_[i].state != 0 &&
                        motor_steer_feedback_[i].state != 0;
    }
  }

  /**
   * @brief 失去控制处理
   *
   */
  void LostCtrl()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Relax();
      motor_steer_[i]->Relax();
    }
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode)
  {
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++)
    {
      pid_wheel_speed_[i].Reset();
      pid_steer_angle_[i].Reset();
      pid_steer_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 将归一化 CMD 输入转换为底盘目标速度，单位为 m/s 和 rad/s
   */
  void UpdateCMD()
  {
    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    const float MAX_CHASSIS_OMEGA = GetMaxChassisOmega();

    /* 先生成目标角速度。 */
    switch (chassis_event_)
    {
      case ChassisMode::RELAX:
        target_omega_ = 0.0f;
        break;
      case ChassisMode::INDEPENDENT:
      case ChassisMode::ROTOR:
        target_omega_ = cmd_data_.z * MAX_CHASSIS_OMEGA;
        break;
      case ChassisMode::FOLLOW:
        /* 增加FOLLOW死区，避免不期望抖动 */
        {
          const float BETA = LibXR::CycleValue<float>(current_yaw_) - 0.0f;
          constexpr float FOLLOW_DEADZONE = static_cast<float>(M_PI) / 90.0f;
          if (std::fabs(BETA) < FOLLOW_DEADZONE)
          {
            target_omega_ = 0.0f;
            pid_follow_.Reset();
          }
          else
          {
            target_omega_ =
                -pid_follow_.Calculate(0.0f, current_yaw_, dt_) * MAX_CHASSIS_OMEGA;
          }
        }
        break;
      default:
        target_omega_ = 0.0f;
        break;
    }

    /* 再生成目标平移速度，保持原有云台坐标变换。 */
    switch (chassis_event_)
    {
      case ChassisMode::RELAX:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case ChassisMode::INDEPENDENT:
        target_vx_ = cmd_data_.x * MAX_WHEEL_LINEAR_SPEED;
        target_vy_ = cmd_data_.y * MAX_WHEEL_LINEAR_SPEED;
        break;
      case ChassisMode::FOLLOW:
      case ChassisMode::ROTOR:
      {
        const float BETA = current_yaw_;
        const float COS_BETA = cosf(BETA);
        const float SIN_BETA = sinf(BETA);
        target_vx_ =
            (COS_BETA * cmd_data_.x - SIN_BETA * cmd_data_.y) * MAX_WHEEL_LINEAR_SPEED;
        target_vy_ =
            (SIN_BETA * cmd_data_.x + COS_BETA * cmd_data_.y) * MAX_WHEEL_LINEAR_SPEED;
      }
      break;
      default:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
    }
  }

  /**
   * @brief 功率控制更新
   *
   */
  void PowerControlUpdate()
  {
    /* 仅提交驱动轮数据；PowerControl 的 motor_count_6020 配置为 0。 */
    for (int i = 0; i < 4; i++)
    {
      motor_data_.rotorspeed_rpm_3508[i] = motor_wheel_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_wheel_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }

    /* 第一次设置: 反馈数据用于 RLS 参数估计 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->CalculatePowerControlParam();

    /* 计算 3508 速度跟踪误差 */
    float speed_error_3508[4];
    for (int i = 0; i < 4; i++)
    {
      float actual_speed = motor_reverse_[i] ? -motor_wheel_feedback_[i].velocity
                                             : motor_wheel_feedback_[i].velocity;
      speed_error_3508[i] = target_speed_[i] - actual_speed;

      motor_data_.output_current_3508[i] =
          std::clamp(wheel_out_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508, speed_error_3508);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    float max_power = static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f)
    {
      max_power = HELM_CHASSIS_MAX_POWER;
    }

    if (power_control_online && boost_mode)
    {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f)
      {
        max_power += 100.0f;
      }
      else if (cap_energy > 0.5f)
      {
        max_power += 70.0f;
      }
      else if (cap_energy > 0.25f)
      {
        max_power += 40.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();
  }

  /**
   * @brief 舵轮底盘逆动力学解算
   * @details 车体速度 PID 输出加速度，分配轮端前馈力矩，单位 N*m
   */
  void DynamicInverseSolution()
  {
    target_ax_ = 0.0f;
    target_ay_ = 0.0f;
    target_alpha_ = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      target_motor_torque_ff_[i] = 0.0f;
    }

    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    const float MAX_CHASSIS_OMEGA = GetMaxChassisOmega();
    if (chassis_event_ == ChassisMode::RELAX ||

        !measured_chassis_velocity_.valid || !std::isfinite(dt_) ||
        !std::isfinite(target_vx_) || !std::isfinite(target_vy_) ||
        !std::isfinite(target_omega_) || dt_ <= 0.0f ||
        !std::isfinite(MAX_WHEEL_LINEAR_SPEED) || MAX_WHEEL_LINEAR_SPEED <= 0.0f ||
        !std::isfinite(MAX_CHASSIS_OMEGA) || MAX_CHASSIS_OMEGA <= 0.0f ||
        !std::isfinite(PARAM.gravity) || PARAM.gravity <= 0.0f ||
        !std::isfinite(PARAM.moment_of_inertia) || PARAM.moment_of_inertia < 0.0f)
    {
      pid_velocity_x_.Reset();
      pid_velocity_y_.Reset();
      pid_omega_.Reset();
      return;
    }

    /* 零速目标时仍闭环底盘速度，以模型力矩制动残余运动。 */
    target_ax_ = pid_velocity_x_.Calculate(target_vx_, measured_chassis_velocity_.x, dt_);
    target_ay_ = pid_velocity_y_.Calculate(target_vy_, measured_chassis_velocity_.y, dt_);
    if (PARAM.moment_of_inertia > 0.0f)
    {
      target_alpha_ =
          pid_omega_.Calculate(target_omega_, measured_chassis_velocity_.omega, dt_);
    }
    else
    {
      pid_omega_.Reset();
    }
    if (!std::isfinite(target_ax_) || !std::isfinite(target_ay_) ||
        !std::isfinite(target_alpha_))
    {
      target_ax_ = 0.0f;
      target_ay_ = 0.0f;
      target_alpha_ = 0.0f;
      pid_velocity_x_.Reset();
      pid_velocity_y_.Reset();
      pid_omega_.Reset();
      return;
    }

    constexpr float GRAVITY_ACCELERATION = 9.80665f;
    const float MASS = PARAM.gravity / GRAVITY_ACCELERATION;
    float sum_radius_squared = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      sum_radius_squared +=
          WHEEL_POSITION.x * WHEEL_POSITION.x + WHEEL_POSITION.y * WHEEL_POSITION.y;
    }
    if (!std::isfinite(sum_radius_squared) || sum_radius_squared <= 1.0e-12f)
    {
      return;
    }

    /* 对称四轮的最小范数分配：
     * Fx_i = m*ax/4 - J*alpha*y_i/S
     * Fy_i = m*ay/4 + J*alpha*x_i/S，S = sum(x_i^2+y_i^2)。 */
    const float TRANSLATION_FORCE_X = MASS * target_ax_ / 4.0f;
    const float TRANSLATION_FORCE_Y = MASS * target_ay_ / 4.0f;
    const float ROTATION_FORCE_SCALE =
        PARAM.moment_of_inertia * target_alpha_ / sum_radius_squared;
    const bool ZERO_CHASSIS_COMMAND = fabsf(target_vx_) <= 1.0e-6f &&
                                      fabsf(target_vy_) <= 1.0e-6f &&
                                      fabsf(target_omega_) <= 1.0e-6f;
    for (int i = 0; i < 4; i++)
    {
      /* 停车时舵角虽锁存，仍需在该实际方向施加制动力矩。 */
      if (steer_angle_held_[i] && !ZERO_CHASSIS_COMMAND)
      {
        continue;
      }
      const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      const float STEER_ANGLE =
          LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
      const float FORCE_X = TRANSLATION_FORCE_X - ROTATION_FORCE_SCALE * WHEEL_POSITION.y;
      const float FORCE_Y = TRANSLATION_FORCE_Y + ROTATION_FORCE_SCALE * WHEEL_POSITION.x;
      /* 实际舵向的驱动正方向为 (sin(a), cos(a))。
       * 投影已包含方向，不再按 motor_reverse 取反或重复乘 cos^3。 */
      const float TORQUE_FF = PARAM.wheel_radius *
                              (FORCE_X * sinf(STEER_ANGLE) + FORCE_Y * cosf(STEER_ANGLE));
      if (std::isfinite(TORQUE_FF))
      {
        target_motor_torque_ff_[i] = TORQUE_FF;
      }
    }
  }

  /**
   * @brief 舵轮底盘逆运动学解算
   * @details 根据底盘目标速度计算各轮目标舵角与电机转子转速
   */
  void InverseKinematicsSolution()
  {
    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    if (MAX_WHEEL_LINEAR_SPEED <= 1.0e-6f)
    {
      for (int i = 0; i < 4; i++)
      {
        target_speed_[i] = 0.0f;
        target_angle_[i] = 0.0f;
        steer_angle_held_[i] = false;
      }
      return;
    }
    const float MOTOR_RPM_PER_MPS = motor_max_speed_ / MAX_WHEEL_LINEAR_SPEED;
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        for (int i = 0; i < 4; i++)
        {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0f;
          steer_angle_held_[i] = false;
        }
        break;
      case (ChassisMode::INDEPENDENT):
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR):
      {
        float max_target_speed = 0.0f;
        for (int i = 0; i < 4; i++)
        {
          const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
          const float WHEEL_VX = target_vx_ - target_omega_ * WHEEL_POSITION.y;
          const float WHEEL_VY = target_vy_ + target_omega_ * WHEEL_POSITION.x;
          const float WHEEL_VECTOR_SPEED =
              sqrtf(WHEEL_VX * WHEEL_VX + WHEEL_VY * WHEEL_VY);
          if (WHEEL_VECTOR_SPEED <= 1.0e-6f)
          {
            /* 仅轮速度近零时锁存舵角，避免零向量的目标舵角无定义。 */
            if (!steer_angle_held_[i])
            {
              target_angle_[i] =
                  LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
              steer_angle_held_[i] = true;
            }
            target_speed_[i] = 0.0f;
          }
          else
          {
            steer_angle_held_[i] = false;
            /* 沿用原有舵角零位：+y 为 0，正角朝 +x。 */
            target_angle_[i] = static_cast<float>(M_PI_2) - atan2f(WHEEL_VY, WHEEL_VX);
            target_speed_[i] = WHEEL_VECTOR_SPEED * MOTOR_RPM_PER_MPS;
            max_target_speed = fmaxf(max_target_speed, target_speed_[i]);
          }
        }
        /* 按最大轮速同比例缩放，保留四轮速度比例并限制至 RPM 上限。 */
        if (max_target_speed > motor_max_speed_)
        {
          const float SPEED_SCALE = motor_max_speed_ / max_target_speed;
          for (int i = 0; i < 4; i++)
          {
            target_speed_[i] *= SPEED_SCALE;
          }
        }
      }
      break;
      default:
        for (int i = 0; i < 4; i++)
        {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0f;
          steer_angle_held_[i] = false;
        }
        break;
    }
  }

  /**
   * @brief 最优舵角选择
   * @details 舵向差超过 90 度时反转驱动轮，减少舵向转动
   */
  void OptimizeWheelAngle()
  {
    for (int i = 0; i < 4; i++)
    {
      if (steer_angle_held_[i])
      {
        /* 静止时保持锁存的物理舵角，不再做 180 度等价翻转。 */
        motor_reverse_[i] = false;
        continue;
      }
      if (fabs(LibXR::CycleValue(motor_steer_feedback_[i].abs_angle - zero_[i]) -
               target_angle_[i]) > M_PI_2)
      {
        motor_reverse_[i] = true;
      }
      else
      {
        motor_reverse_[i] = false;
      }
    }
  }

  /**
   * @brief 舵向误差对应的驱动轮速度补偿
   */
  void CompensateWheelSpeed()
  {
    /* 最优角确定后按舵向误差抑制轮速，减少转舵时的侧向拖拽。 */
    for (int i = 0; i < 4; i++)
    {
      const LibXR::CycleValue<float> CURRENT_ANGLE(motor_steer_feedback_[i].abs_angle -
                                                   zero_[i]);
      const LibXR::CycleValue<float> OPTIMAL_ANGLE(
          target_angle_[i] + (motor_reverse_[i] ? static_cast<float>(M_PI) : 0.0f));
      const float STEER_ERROR = OPTIMAL_ANGLE - CURRENT_ANGLE;
      const float ALIGNMENT = fmaxf(0.0f, cosf(STEER_ERROR));
      target_speed_[i] *= ALIGNMENT * ALIGNMENT * ALIGNMENT;
    }
  }

  /**
   * @brief 计算舵向和驱动轮 PID 输出
   */
  void CalculateMotorCurrent()
  {
    /* 轮速 PID 跟踪观测底盘速度在当前实际舵向上的投影。 */
    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    const float MOTOR_RPM_PER_MPS = measured_chassis_velocity_.valid &&
                                            std::isfinite(MAX_WHEEL_LINEAR_SPEED) &&
                                            MAX_WHEEL_LINEAR_SPEED > 1.0e-6f
                                        ? motor_max_speed_ / MAX_WHEEL_LINEAR_SPEED
                                        : 0.0f;
    /* 轮速目标的符号由实际舵向决定，反转状态只影响舵向控制。 */
    for (int i = 0; i < 4; i++)
    {
      float observed_motor_rpm = 0.0f;
      if (MOTOR_RPM_PER_MPS > 0.0f)
      {
        const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
        const float STEER_ANGLE =
            LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
        const float WHEEL_VX = measured_chassis_velocity_.x -
                               measured_chassis_velocity_.omega * WHEEL_POSITION.y;
        const float WHEEL_VY = measured_chassis_velocity_.y +
                               measured_chassis_velocity_.omega * WHEEL_POSITION.x;
        observed_motor_rpm =
            (WHEEL_VX * sinf(STEER_ANGLE) + WHEEL_VY * cosf(STEER_ANGLE)) *
            MOTOR_RPM_PER_MPS;
        if (!std::isfinite(observed_motor_rpm))
        {
          observed_motor_rpm = 0.0f;
        }
      }
      wheel_out_[i] = pid_wheel_speed_[i].Calculate(
          observed_motor_rpm, motor_wheel_feedback_[i].velocity, dt_);
      if (motor_reverse_[i])
      {
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            LibXR::CycleValue<float>(target_angle_[i] + static_cast<float>(M_PI) +
                                     zero_[i]),
            motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity * static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
      else
      {
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            target_angle_[i] + zero_[i], motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity * static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
      /* 与轮速 PID 同为减速器输出轴扭矩 N*m，交给现有功率控制限幅。 */
      wheel_out_[i] += target_motor_torque_ff_[i];
    }
  }

  /**
   * @brief 输出控制命令到电机
   *
   */
  void Output()
  {
    if (power_control_data_.is_power_limited)
    {
      /* 功率限制只替换驱动轮输出，舵向保留 PID 输出和自身扭矩限幅。 */
      for (int i = 0; i < 4; i++)
      {
        wheel_out_[i] = std::clamp(power_control_data_.new_output_current_3508[i] /
                                       M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                                   -4.4f, 4.4f);
      }
    }

    if (chassis_event_ == (ChassisMode::RELAX))
    {
      LostCtrl();
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_cmd_[i].torque = std::clamp(wheel_out_[i], -4.4f, 4.4f);
        motor_steer_cmd_[i].torque = std::clamp(steer_out_[i], -2.5f, 2.5f);
      }
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_[i]->Control(motor_wheel_cmd_[i]);
        motor_steer_[i]->Control(motor_steer_cmd_[i]);
      }
    }
  }

 private:
  const ChassisParam PARAM;

  struct WheelPosition
  {
    float x;
    float y;
  };

  WheelPosition GetWheelPosition(uint8_t wheel_index) const
  {
    const float COMPONENT = PARAM.wheel_to_center * 0.70710678118f;
    switch (wheel_index)
    {
      case 0:
        return {-COMPONENT, -COMPONENT};  // Left rear, quadrant III
      case 1:
        return {COMPONENT, -COMPONENT};  // Right rear, quadrant IV
      case 2:
        return {COMPONENT, COMPONENT};  // Right front, quadrant I
      case 3:
        return {-COMPONENT, COMPONENT};  // Left front, quadrant II
      default:
        return {0.0f, 0.0f};
    }
  }

  float GetMaxWheelLinearSpeed() const
  {
    if (PARAM.wheel_radius <= 1.0e-6f || PARAM.reduction_ratio <= 1.0e-6f ||
        motor_max_speed_ <= 1.0e-6f)
    {
      return 0.0f;
    }
    return motor_max_speed_ * static_cast<float>(LibXR::TWO_PI) * PARAM.wheel_radius /
           (60.0f * PARAM.reduction_ratio);
  }

  float GetMaxChassisOmega() const
  {
    /* 纯旋转时轮心线速度为 |omega|*R，R 已是轮心到车体中心的距离。 */
    return PARAM.wheel_to_center > 1.0e-6f
               ? GetMaxWheelLinearSpeed() / PARAM.wheel_to_center
               : 0.0f;
  }

  /**
   * @brief 舵轮底盘正运动学解算
   * @details 根据四轮转速和舵角解算当前底盘速度
   */
  void SelfResolution()
  {
    measured_chassis_velocity_ = {};
    if (!feedback_valid_ || PARAM.wheel_to_center <= 0.0f || PARAM.wheel_radius <= 0.0f ||
        PARAM.reduction_ratio <= 0.0f)
    {
      return;
    }

    const float RPM_TO_LINEAR_SPEED = static_cast<float>(LibXR::TWO_PI) *
                                      PARAM.wheel_radius /
                                      (60.0f * PARAM.reduction_ratio);
    float sum_vx = 0.0f;
    float sum_vy = 0.0f;
    float sum_rotation = 0.0f;
    float sum_radius_squared = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      /* 舵向零位沿 +y，正角朝 +x；电机 reverse 已在反馈解码中处理。 */
      const float STEER_ANGLE =
          LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
      const float WHEEL_SPEED = motor_wheel_feedback_[i].velocity * RPM_TO_LINEAR_SPEED;
      if (!std::isfinite(STEER_ANGLE) || !std::isfinite(WHEEL_SPEED))
      {
        return;
      }
      const float WHEEL_VX = WHEEL_SPEED * sinf(STEER_ANGLE);
      const float WHEEL_VY = WHEEL_SPEED * cosf(STEER_ANGLE);
      sum_vx += WHEEL_VX;
      sum_vy += WHEEL_VY;
      sum_rotation += WHEEL_POSITION.x * WHEEL_VY - WHEEL_POSITION.y * WHEEL_VX;
      sum_radius_squared +=
          WHEEL_POSITION.x * WHEEL_POSITION.x + WHEEL_POSITION.y * WHEEL_POSITION.y;
    }
    if (sum_radius_squared <= 1.0e-12f)
    {
      return;
    }
    measured_chassis_velocity_.x = sum_vx / 4.0f;
    measured_chassis_velocity_.y = sum_vy / 4.0f;
    measured_chassis_velocity_.omega = sum_rotation / sum_radius_squared;
    measured_chassis_velocity_.valid = std::isfinite(measured_chassis_velocity_.x) &&
                                       std::isfinite(measured_chassis_velocity_.y) &&
                                       std::isfinite(measured_chassis_velocity_.omega);
    if (!measured_chassis_velocity_.valid)
    {
      measured_chassis_velocity_ = {};
    }
  }

  ChassisVelocity measured_chassis_velocity_{};
  bool feedback_valid_ = false;
  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;
  float target_ax_ = 0.0f;    /* m/s^2 */
  float target_ay_ = 0.0f;    /* m/s^2 */
  float target_alpha_ = 0.0f; /* rad/s^2 */
  float target_motor_torque_ff_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  bool motor_reverse_[4]{false, false, false, false};
  LibXR::CycleValue<float> zero_[4] = {1.10063124, 2.05323339, 3.2244277, 0.979446769};
  float current_yaw_ = 0.0f;
  float delta_yaw_ = 0.0f;

  /* 驱动电机转子目标转速 RPM，舵角以 +y 为零位、单位 rad。 */
  float target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::CycleValue<float> target_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool steer_angle_held_[4] = {false, false, false, false};

  /* 输出的电流值 */
  float wheel_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_angle_[4] = {0.0, 0.0, 0.0, 0.0};

  float motor_max_speed_ = 9000.0;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;
  Motor* motor_steer_0_;
  Motor* motor_steer_1_;
  Motor* motor_steer_2_;
  Motor* motor_steer_3_;

  // Wheel order: 0=LB (QIII), 1=RB (QIV), 2=RF (QI), 3=LF (QII).
  // Coordinates use +x right and +y forward; indices increase counterclockwise.
  Motor* motor_wheel_[4] = {motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                            motor_wheel_3_};
  Motor* motor_steer_[4] = {motor_steer_0_, motor_steer_1_, motor_steer_2_,
                            motor_steer_3_};

  Motor::Feedback motor_wheel_feedback_[4]{};
  Motor::Feedback motor_steer_feedback_[4]{};

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_angle_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};

  ChassisTopicNames topic_names_{};
  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  Motor::MotorCmd motor_wheel_cmd_[4]{};
  Motor::MotorCmd motor_steer_cmd_[4]{};

  MotorData motor_data_ = {};
  PowerControl* power_control_;
  PowerControlData power_control_data_;

  Referee::ChassisPack referee_chassis_pack_{};
  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  LibXR::Topic topic_delta_yaw_ = LibXR::Topic::CreateTopic<float>("delta_yaw");
  ChassisMode chassis_event_ = ChassisMode::RELAX;
};
