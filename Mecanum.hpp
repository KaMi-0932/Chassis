#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 麦轮底盘控制实现，由 Chassis 模板使用 / Mecanum chassis controller used by the Chassis template
depends: []
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "SuperPower.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */

#define MECANUM_MOTOR_MAX_OMEGA 52.0f /* 麦轮输出轴最大角速度 rad/s */
#define MECANUM_CHASSIS_MAX_POWER 70  /* 麦轮默认功率上限 W */

template <typename ChassisType>
class Chassis;

/**
 * @brief 麦轮底盘控制实现，由 Chassis 模板使用。
 *        Mecanum chassis controller used by the Chassis template.
 */
class Mecanum
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
    float gravity = 0.0f;  ///< 底盘重力 (N)，非正值关闭上坡重力前馈
    ///< Chassis weight (N); non-positive values disable gravity feedforward
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
    float friction_compensation_0 = 0.0f;  ///< 0 号轮输出轴摩擦补偿力矩 (N*m)
    float friction_compensation_1 = 0.0f;  ///< 1 号轮输出轴摩擦补偿力矩 (N*m)
    float friction_compensation_2 = 0.0f;  ///< 2 号轮输出轴摩擦补偿力矩 (N*m)
    float friction_compensation_3 = 0.0f;  ///< 3 号轮输出轴摩擦补偿力矩 (N*m)
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
    FOLLOW,  ///< 跟随：底盘跟随云台 yaw，平移相对云台坐标系
    ///< Follow: the chassis follows the gimbal yaw, translation in the gimbal frame
    TRACK_START  ///< 兼容旧配置的保留值，不再控制履带
    ///< Reserved for legacy configurations; no track control is performed
  };
  /**
   * @brief 构造麦轮底盘控制对象，创建控制线程。
   *        Construct the mecanum chassis controller and create the control thread.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param motor_steer_1 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param motor_steer_2 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param motor_steer_3 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
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
   * @param pid_steer_angle_0 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_angle_1 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_angle_2 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_angle_3 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_0 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_1 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_2 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_3 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param thread_priority 控制线程优先级。
   *                        Control thread priority.
   * @param topic_names 订阅的 Topic 名称。
   *                    Names of the subscribed Topics.
   */
  Mecanum(

      Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
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
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,
      ChassisTopicNames topic_names = {})
      /*
       * 麦轮编号，箭头为轮子正方向
       *
       *                 ↑ y
       *        wheel0   │   wheel3
       *          ↑      │      ↓
       *                 │
       * ────────────────┼──────────────→ x
       *                 │
       *          ↑      │      ↓
       *        wheel1   │   wheel2
       */
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        pid_follow_(pid_follow),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1, pid_wheel_speed_2,
                         pid_wheel_speed_3},
        cmd_(cmd),
        power_control_(power_control),
        referee_(referee)
  {
    topic_names_ = topic_names;
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    UNUSED(motor_steer_0);
    UNUSED(pid_steer_speed_0);
    UNUSED(pid_steer_speed_1);
    UNUSED(pid_steer_speed_2);
    UNUSED(pid_steer_speed_3);
    UNUSED(pid_steer_angle_0);
    UNUSED(pid_steer_angle_1);
    UNUSED(pid_steer_angle_2);
    UNUSED(pid_steer_angle_3);

    for (int i = 0; i < 4; i++)
    {
      motor_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_cmd_[i].torque = 0.0f;
      motor_cmd_[i].position = 0.0f;
      motor_cmd_[i].velocity = 0.0f;
      motor_cmd_[i].kp = 0.0f;
      motor_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "MecanumChassisThread", task_stack_depth,
                   thread_priority);
    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->mutex_.Lock();
          mecanum->chassis_event_ = ChassisMode::RELAX;
          mecanum->mutex_.Unlock();
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->mutex_.Lock();
          mecanum->chassis_event_ = ChassisMode::RELAX;
          mecanum->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    void (*InitUi)(Mecanum*) = [](Mecanum* mecanum) { mecanum->InitUi(); };
    auto InitUi_ = LibXR::Timer::CreateTask(InitUi, this, 52);
    LibXR::Timer::Add(InitUi_);
    LibXR::Timer::Start(InitUi_);
  }

  /**
   * @brief 麦轮底盘控制线程函数。
   *        Control thread function of the mecanum chassis.
   *
   * @details 线程主循环：接收控制命令，执行运动学解算并输出控制量。
   *          Main loop: receives the control command, solves the kinematics and sends the
   *          outputs.
   *
   * @param mecanum Mecanum 对象指针。
   *                Pointer to the Mecanum object.
   */
  static void ThreadFunction(Mecanum* mecanum)
  {
    mecanum->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(
        mecanum->topic_names_.chassis_cmd);
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        mecanum->topic_names_.chassis_ref);
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");
    LibXR::Topic::ASyncSubscriber<float> pitchmotor_angle_suber("pitchmotor_angle");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        mecanum->topic_names_.gimbal_euler);

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();
    pitchmotor_angle_suber.StartWaiting();
    euler_suber.StartWaiting();

    mecanum->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    mecanum->mutex_.Unlock();

    while (true)
    {
      if (cmd_suber.Available())
      {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available())
      {
        mecanum->referee_chassis_pack_ = referee_suber.GetData();
        mecanum->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available())
      {
        mecanum->current_yaw_ = LibXR::CycleValue<float>(yawmotor_angle_suber.GetData() -
                                                         mecanum->yawmotor_zero_);
        mecanum->gravity_yaw_rx_time_ = LibXR::Timebase::GetMilliseconds();
        mecanum->gravity_yaw_received_ = true;
        yawmotor_angle_suber.StartWaiting();
      }

      if (pitchmotor_angle_suber.Available())
      {
        mecanum->gravity_pitch_motor_angle_ = pitchmotor_angle_suber.GetData();
        mecanum->gravity_pitch_rx_time_ = LibXR::Timebase::GetMilliseconds();
        mecanum->gravity_pitch_received_ = true;
        pitchmotor_angle_suber.StartWaiting();
      }

      if (euler_suber.Available())
      {
        mecanum->gravity_euler_ = euler_suber.GetData();
        mecanum->gravity_euler_rx_time_ = LibXR::Timebase::GetMilliseconds();
        mecanum->gravity_euler_received_ = true;
        euler_suber.StartWaiting();
      }

      mecanum->mutex_.Lock();
      mecanum->Update();
      mecanum->UpdateCMD();
      mecanum->SelfResolution();
      mecanum->InverseKinematicsSolution();
      mecanum->DynamicInverseSolution();
      mecanum->GravityFeedforward();
      mecanum->FrictionCompensation();
      mecanum->CalculateMotorCurrent();
      mecanum->PowerControlUpdate();
      mecanum->mutex_.Unlock();
      mecanum->OutputToDynamics();

      mecanum->thread_.SleepUntil(last_time, 2);
    }
  }

  /**
   * @brief 更新电机状态。
   *        Update the motor states.
   *
   * @details 记录采样间隔，更新四个驱动轮电机并读取反馈。
   *          Records the sampling interval, updates the four wheel motors and reads their
   *          feedback.
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
      motor_feedback_[i] = motor_wheel_[i]->GetFeedback();
      feedback_valid_ = feedback_valid_ && WHEEL_STATUS == LibXR::ErrorCode::OK &&
                        motor_feedback_[i].state != 0;
    }
  }
  /**
   * @brief 设置底盘模式并复位速度与轮速 PID，由 Chassis 外壳调用。
   *        Set the chassis mode and reset the velocity and wheel-speed PIDs;
   *        called by the Chassis shell.
   *
   * @param mode 新模式，取 ChassisMode 的值。
   *             New mode, a value of ChassisMode.
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
    }
    mutex_.Unlock();
  }

  /**
   * @brief 从 CMD 命令更新目标角速度与目标平移速度。
   *        Update the target angular velocity and the target translation velocity from
   *        the CMD command.
   */
  void UpdateCMD()
  {
    float max_v = PARAM.wheel_radius * MECANUM_MOTOR_MAX_OMEGA;

    /* 先生成目标角速度 */
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;

      case (ChassisMode::INDEPENDENT):
        target_omega_ = -max_v * cmd_data_.z / PARAM.wheel_to_center;
        break;

      case (ChassisMode::ROTOR):
        this->target_omega_ = static_cast<float>(max_v / PARAM.wheel_to_center);
        break;

      case (ChassisMode::FOLLOW):
        target_omega_ = -pid_follow_.Calculate(0.0f, -current_yaw_, dt_);
        break;

      default:
        target_omega_ = 0.0f;
        break;
    }

    /* 再生成目标平移速度 */
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::ROTOR):
      case (ChassisMode::FOLLOW):
      {
        float beta = -current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = (cos_beta * cmd_data_.x * max_v + sin_beta * cmd_data_.y * max_v);
        target_vy_ = (-sin_beta * cmd_data_.x * max_v + cos_beta * cmd_data_.y * max_v);
      }
      break;
      case (ChassisMode::INDEPENDENT):
      {
        target_vx_ = cmd_data_.x * max_v;
        target_vy_ = cmd_data_.y * max_v;
      }
      break;
      default:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
    }

    /* 小陀螺模式下根据平移输入与功率状态动态调整旋转速度 */
    float rotor_translation_scale = 1.0f;
    if (chassis_event_ == ChassisMode::ROTOR)
    {
      float translation_magnitude =
          sqrtf(target_vx_ * target_vx_ + target_vy_ * target_vy_);
      float translation_ratio = 0.0f;
      if (max_v > 1e-3f)
      {
        translation_ratio = std::clamp(translation_magnitude / max_v, 0.0f, 1.0f);
      }
      rotor_translation_scale =
          1.0f - (1.0f - PARAM.rotor_speed_scale) * translation_ratio;
      target_omega_ *= rotor_translation_scale * rotor_dynamic_scale_;
    }
  }

  /**
   * @brief 麦轮底盘正运动学解算。
   *        Forward kinematics of the mecanum chassis.
   *
   * @details 由四个麦轮的角速度解算底盘的运动状态。
   *          Solves the chassis motion from the angular velocities of the four wheels.
   */
  void SelfResolution()
  {
    now_vx_ = (motor_feedback_[0].omega / PARAM.reduction_ratio -
               motor_feedback_[1].omega / PARAM.reduction_ratio -
               motor_feedback_[2].omega / PARAM.reduction_ratio +
               motor_feedback_[3].omega / PARAM.reduction_ratio) *
              PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
               motor_feedback_[1].omega / PARAM.reduction_ratio -
               motor_feedback_[2].omega / PARAM.reduction_ratio -
               motor_feedback_[3].omega / PARAM.reduction_ratio) *
              PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
                  motor_feedback_[1].omega / PARAM.reduction_ratio +
                  motor_feedback_[2].omega / PARAM.reduction_ratio +
                  motor_feedback_[3].omega / PARAM.reduction_ratio) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 麦轮底盘逆运动学解算。
   *        Inverse kinematics of the mecanum chassis.
   *
   * @details 由目标底盘速度 (vx, vy, ω) 计算四个麦轮的目标角速度。
   *          Computes the target angular velocity of the four wheels from the target
   *          chassis velocity (vx, vy, ω).
   */
  void InverseKinematicsSolution()
  {
    target_motor_omega_[0] =
        (target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[1] =
        (-target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[2] =
        (-target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[3] =
        (target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
  }

  /**
   * @brief 计算轮速 PID 输出并与动力学、上坡重力前馈力矩合成。
   *        Compute the wheel-speed PID outputs and combine them with the feedforward
   *        torque and gravity feedforward.
   */
  void CalculateMotorCurrent()
  {
    if (chassis_event_ == ChassisMode::RELAX)
    {
      LostCtrl();
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        target_motor_current_[i] = pid_wheel_speed_[i].Calculate(
            target_motor_omega_[i], motor_feedback_[i].omega / PARAM.reduction_ratio,
            dt_);
      }
      /* 三项均为减速器输出轴扭矩，合成后统一交给功率控制限幅。 */
      for (int i = 0; i < 4; i++)
      {
        output_[i] = target_motor_force_[i] * PARAM.wheel_radius +
                     target_motor_current_[i] + gravity_torque_ff_[i] +
                     friction_torque_ff_[i];
      }
    }
  }

  /**
   * @brief 更新功率控制与小陀螺动态缩放。
   *        Update power control and the dynamic spin scale.
   *
   * @details 提交反馈与期望输出，计算功率上限并读取限幅结果。
   *          Submits the feedback and the requested outputs, computes the power limit and
   *          reads the limited result.
   */
  void PowerControlUpdate()
  {
    /* 采样当前反馈电流和转速供功率模型参数估计使用 */
    for (int i = 0; i < 4; i++)
    {
      motor_data_.rotorspeed_rpm_3508[i] = motor_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->CalculatePowerControlParam();

    float speed_error[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};

    /* 写入四路期望电流供限功率使用 */
    for (int i = 0; i < 4; i++)
    {
      speed_error[i] =
          target_motor_omega_[i] - motor_feedback_[i].omega / PARAM.reduction_ratio;
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508, speed_error);
    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    /* 裁判系统离线或上限异常时回退到本地默认功率上限 */
    float max_power = static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f)
    {
      max_power = MECANUM_CHASSIS_MAX_POWER;
    }

    /* BOOST 模式按电容能量分档提升可用功率上限 */
    if (power_control_online && boost_mode)
    {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f)
      {
        max_power += 300.0f;
      }
      else if (cap_energy > 0.5f)
      {
        max_power += 200.0f;
      }
      else if (cap_energy > 0.25f)
      {
        max_power += 100.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();

    /* 受限程度估计为限幅后电流总量除以请求电流总量 */
    float req_current_abs_sum = 0.0f;
    float lim_current_abs_sum = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      float req_current_abs = fabsf(motor_data_.output_current_3508[i]);
      float lim_current_abs = power_control_data_.is_power_limited
                                  ? fabsf(power_control_data_.new_output_current_3508[i])
                                  : req_current_abs;
      req_current_abs_sum += req_current_abs;
      lim_current_abs_sum += lim_current_abs;
    }

    float power_limit_ratio = 1.0f;
    if (req_current_abs_sum > 1e-3f)
    {
      power_limit_ratio =
          std::clamp(lim_current_abs_sum / req_current_abs_sum, 0.0f, 1.0f);
    }

    /* 将裁判缓冲能量映射为缩放因子，裁判系统离线时保持 1.0 */
    float buffer_scale = 1.0f;
    if (referee_online)
    {
      float buffer_range =
          std::max(PARAM.rotor_buffer_high_j - PARAM.rotor_buffer_low_j, 1.0f);
      float referee_buffer_j = static_cast<float>(referee_chassis_pack_.power_buffer);
      float buffer_norm = std::clamp(
          (referee_buffer_j - PARAM.rotor_buffer_low_j) / buffer_range, 0.0f, 1.0f);
      buffer_scale = PARAM.rotor_omega_min_scale +
                     (1.0f - PARAM.rotor_omega_min_scale) * buffer_norm;
    }

    /* 合成目标缩放并做一阶低通抑制跳变 */
    float limit_scale =
        power_control_data_.is_power_limited
            ? std::clamp(power_limit_ratio, PARAM.rotor_omega_min_scale, 1.0f)
            : 1.0f;
    float target_dynamic_scale =
        std::clamp(buffer_scale * limit_scale, PARAM.rotor_omega_min_scale, 1.0f);
    float lpf_alpha = std::clamp(PARAM.rotor_scale_lpf_alpha, 0.0f, 1.0f);
    rotor_dynamic_scale_ += (target_dynamic_scale - rotor_dynamic_scale_) * lpf_alpha;
    rotor_dynamic_scale_ =
        std::clamp(rotor_dynamic_scale_, PARAM.rotor_omega_min_scale, 1.0f);

    /* 仅在小陀螺模式保留缩放，其他模式复位为 1.0 */
    if (chassis_event_ != ChassisMode::ROTOR)
    {
      rotor_dynamic_scale_ = 1.0f;
    }
  }

  /**
   * @brief 麦轮底盘逆动力学解算。
   *        Inverse dynamics of the mecanum chassis.
   *
   * @details 以运动学正解得到的当前状态与目标状态做 PID，得到各轮的前馈力。
   *          Runs PIDs between the state from the forward kinematics and the target state
   *          to obtain the feedforward force of each wheel.
   */
  void DynamicInverseSolution()
  {
    float force_x = pid_velocity_x_.Calculate(target_vx_, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, now_vy_, dt_);
    float force_z = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    /* 按麦轮受力方向分配前馈力 */
    target_motor_force_[0] = (force_x + force_y + force_z) / 4;
    target_motor_force_[1] = (-force_x + force_y + force_z) / 4;
    target_motor_force_[2] = (-force_x - force_y + force_z) / 4;
    target_motor_force_[3] = (force_x - force_y + force_z) / 4;
  }

  /**
   * @brief 沿用舵轮的姿态重建、滤波与坡度回差，按麦轮驱动方向分配重力补偿。
   * @details g_chassis = Rz(yaw) * Rx(pitch) * R_axes * g_ahrs。
   *          R_axes 将 AHRS 的 x 前、y 左、z 上转为云台关节使用的 x 右、y 前、z 上。
   *          BMI088 rotation 只补偿 C 板安装朝向；AHRS yaw 不参与重力重建。
   */
  void GravityFeedforward()
  {
    for (int i = 0; i < 4; i++)
    {
      gravity_torque_ff_[i] = 0.0f;
      gravity_ff_debug_.torque[i] = 0.0f;
    }
    gravity_ff_debug_.active = false;

    const auto NOW_MS = LibXR::Timebase::GetMilliseconds();
    auto is_fresh = [NOW_MS](bool received, LibXR::MillisecondTimestamp rx_time)
    { return received && (NOW_MS - rx_time).ToMillisecond() <= GRAVITY_FF_TIMEOUT_MS; };
    const bool INPUT_VALID = is_fresh(gravity_euler_received_, gravity_euler_rx_time_) &&
                             is_fresh(gravity_yaw_received_, gravity_yaw_rx_time_) &&
                             is_fresh(gravity_pitch_received_, gravity_pitch_rx_time_);
    gravity_ff_debug_.input_valid = INPUT_VALID;
    const bool CONTROL_ENABLED = chassis_event_ == ChassisMode::INDEPENDENT ||
                                 chassis_event_ == ChassisMode::ROTOR ||
                                 chassis_event_ == ChassisMode::FOLLOW;
    if (!CONTROL_ENABLED || !feedback_valid_ || !INPUT_VALID || !std::isfinite(dt_) ||
        dt_ <= 0.0f || !std::isfinite(PARAM.gravity) || PARAM.gravity <= 0.0f ||
        !std::isfinite(PARAM.wheel_radius) || PARAM.wheel_radius <= 0.0f ||
        !std::isfinite(PARAM.reduction_ratio) || PARAM.reduction_ratio <= 0.0f)
    {
      ResetGravityFeedforward();
      return;
    }

    /* 从 AHRS 发布坐标系的 ZYX 欧拉角得到重力，只使用 roll/pitch。 */
    const float ROLL = gravity_euler_.Roll();
    const float PITCH = gravity_euler_.Pitch();
    const float G_AHRS_X = GRAVITY_ACCELERATION * sinf(PITCH);
    const float G_AHRS_Y = -GRAVITY_ACCELERATION * cosf(PITCH) * sinf(ROLL);
    const float G_AHRS_Z = -GRAVITY_ACCELERATION * cosf(PITCH) * cosf(ROLL);

    /* 芯片 x 朝前、y 朝左、z 朝上 -> 底盘约定 x 右、y 前、z 上。 */
    const float G_P_X = -G_AHRS_Y;
    const float G_P_Y = G_AHRS_X;
    const float G_P_Z = G_AHRS_Z;

    /* pitch 关节绕 +x，正角抬头。 */
    const float PITCH_JOINT = GRAVITY_FF_PITCH_SIGN * gravity_pitch_motor_angle_;
    const float CP = cosf(PITCH_JOINT);
    const float SP = sinf(PITCH_JOINT);
    const float G_Y_X = G_P_X;
    const float G_Y_Y = CP * G_P_Y - SP * G_P_Z;
    const float G_Y_Z = SP * G_P_Y + CP * G_P_Z;

    /* 与 FOLLOW 模式中云台系到底盘系的旋转方向一致。 */
    const float YAW_JOINT = GRAVITY_FF_YAW_SIGN * current_yaw_;
    const float CY = cosf(YAW_JOINT);
    const float SY = sinf(YAW_JOINT);
    const float G_BODY[3] = {CY * G_Y_X - SY * G_Y_Y, SY * G_Y_X + CY * G_Y_Y, G_Y_Z};
    if (!std::isfinite(G_BODY[0]) || !std::isfinite(G_BODY[1]) ||
        !std::isfinite(G_BODY[2]))
    {
      ResetGravityFeedforward();
      return;
    }

    const float RC = 1.0f / (static_cast<float>(LibXR::TWO_PI) * GRAVITY_FF_LPF_HZ);
    const float ALPHA = dt_ / (RC + dt_);
    for (int axis = 0; axis < 3; axis++)
    {
      gravity_body_filtered_[axis] =
          gravity_filter_initialized_
              ? gravity_body_filtered_[axis] +
                    ALPHA * (G_BODY[axis] - gravity_body_filtered_[axis])
              : G_BODY[axis];
      gravity_ff_debug_.g_body[axis] = gravity_body_filtered_[axis];
    }
    gravity_filter_initialized_ = true;

    float a_gx = -gravity_body_filtered_[0];
    float a_gy = -gravity_body_filtered_[1];
    const float A_G = hypotf(a_gx, a_gy);
    const float G_NORM =
        sqrtf(A_G * A_G + gravity_body_filtered_[2] * gravity_body_filtered_[2]);
    const float TILT =
        G_NORM > 1.0e-3f ? asinf(std::clamp(A_G / G_NORM, 0.0f, 1.0f)) : 0.0f;
    gravity_ff_debug_.tilt_deg = TILT * 180.0f / static_cast<float>(M_PI);

    /* 超过 3 度开启完整补偿，回落到 2 度以下关闭。 */
    gravity_ff_engaged_ =
        TILT > (gravity_ff_engaged_ ? GRAVITY_FF_OFF_RAD : GRAVITY_FF_ON_RAD);
    if (!gravity_ff_engaged_)
    {
      gravity_ff_debug_.a_g[0] = 0.0f;
      gravity_ff_debug_.a_g[1] = 0.0f;
      return;
    }

    const float A_G_LIMIT = GRAVITY_FF_FRICTION * GRAVITY_ACCELERATION;
    if (A_G > A_G_LIMIT)
    {
      a_gx *= A_G_LIMIT / A_G;
      a_gy *= A_G_LIMIT / A_G;
    }
    gravity_ff_debug_.a_g[0] = a_gx;
    gravity_ff_debug_.a_g[1] = a_gy;

    const float MASS = PARAM.gravity / GRAVITY_ACCELERATION;
    const float FORCE_X = MASS * a_gx;
    const float FORCE_Y = MASS * a_gy;
    /* 与麦轮逆动力学使用相同分配，补偿合力不引入 yaw 力矩。 */
    gravity_torque_ff_[0] = (FORCE_X + FORCE_Y) * PARAM.wheel_radius / 4.0f;
    gravity_torque_ff_[1] = (-FORCE_X + FORCE_Y) * PARAM.wheel_radius / 4.0f;
    gravity_torque_ff_[2] = (-FORCE_X - FORCE_Y) * PARAM.wheel_radius / 4.0f;
    gravity_torque_ff_[3] = (FORCE_X - FORCE_Y) * PARAM.wheel_radius / 4.0f;
    for (int i = 0; i < 4; i++)
    {
      gravity_ff_debug_.torque[i] = gravity_torque_ff_[i];
    }
    gravity_ff_debug_.active = true;
  }

  /**
   * @brief 清除重力前馈输出、滤波和坡度回差状态。
   */
  void ResetGravityFeedforward()
  {
    gravity_filter_initialized_ = false;
    gravity_ff_engaged_ = false;
    gravity_ff_debug_.active = false;
    for (int i = 0; i < 4; i++)
    {
      gravity_torque_ff_[i] = 0.0f;
      gravity_ff_debug_.torque[i] = 0.0f;
    }
    for (int axis = 0; axis < 3; axis++)
    {
      gravity_body_filtered_[axis] = 0.0f;
      gravity_ff_debug_.g_body[axis] = 0.0f;
    }
    gravity_ff_debug_.a_g[0] = 0.0f;
    gravity_ff_debug_.a_g[1] = 0.0f;
    gravity_ff_debug_.tilt_deg = 0.0f;
  }
  /**
   * @brief 按各轮目标方向补偿输出轴摩擦力矩；停车和反向制动时关闭。
   */
  void FrictionCompensation()
  {
    const float COMPENSATION[4] = {
        PARAM.friction_compensation_0, PARAM.friction_compensation_1,
        PARAM.friction_compensation_2, PARAM.friction_compensation_3};
    for (int i = 0; i < 4; i++)
    {
      friction_torque_ff_[i] = 0.0f;
      if (chassis_event_ == ChassisMode::RELAX || !feedback_valid_ ||
           COMPENSATION[i] <= 0.0f)
      {
        continue;
      }

      /* 目标和反馈均为减速器输出轴角速度 (rad/s)。 */
      const float ACTUAL = motor_feedback_[i].omega / PARAM.reduction_ratio;
      if (std::fabs(target_motor_omega_[i]) <= FRICTION_OMEGA_DEADBAND)
      {
        continue;
      }
      const float sign = target_motor_omega_[i] > 0.0f ? 1.0f : -1.0f;
      if (ACTUAL * sign < -FRICTION_OMEGA_DEADBAND)
      {
        continue;
      }
      friction_torque_ff_[i] = COMPENSATION[i] * sign;
    }
  }

  /**
   * @brief 麦轮底盘动力学输出。
   *        Dynamics output of the mecanum chassis.
   *
   * @details 限幅后向四个麦轮下发力矩指令。
   *          Limits the torques and sends them to the four wheels.
   */
  void OutputToDynamics()
  {
    if (power_control_data_.is_power_limited)
    {
      if (gravity_ff_debug_.active)
      {
        ++gravity_ff_debug_.power_limited_count;
      }
      for (int i = 0; i < 4; i++)
      {
        output_[i] = std::clamp(power_control_data_.new_output_current_3508[i] /
                                    M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                                -6.0f, 6.0f);
      }
    }
    if (chassis_event_ == ChassisMode::RELAX)
    {
      LostCtrl();
      return;
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        motor_cmd_[i].torque = std::clamp(output_[i], -6.0f, 6.0f);
      }
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_[i]->Control(motor_cmd_[i]);
      }
    }
  }
  /**
   * @brief 失去控制时使全部电机放松。
   *        Relax all motors when control is lost.
   */
  void LostCtrl()
  {
    ResetGravityFeedforward();
    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Relax();
    }
  }
  void InitUi()
  {
    const uint16_t id = referee_->GetRobotID();
    const uint16_t client = referee_->GetClientID(id);
    Referee::UIFigureOp ADD_OP = Referee::UIFigureOp::UI_OP_MODIFY;
    if (this->ui_dyn_step_ % 4 == 0)
    {
      ADD_OP = Referee::UIFigureOp::UI_OP_ADD;
      ui_dyn_step_ = 0;
    }

    switch (ui_step_)
    {
      case 0:
      {
        const char* mode_str = "RELX";
        switch (chassis_event_)
        {
          case ChassisMode::RELAX:
            mode_str = "RELX";
            current_mode_ = ChassisMode::RELAX;
            break;
          case ChassisMode::FOLLOW:
            mode_str = "FOLW";
            current_mode_ = ChassisMode::FOLLOW;
            break;
          case ChassisMode::ROTOR:
            mode_str = "ROTO";
            current_mode_ = ChassisMode::ROTOR;
            break;
          default:
            mode_str = "RELX";
            current_mode_ = ChassisMode::RELAX;
            break;
        }
        if (current_mode_ != last_ui_mode_ or ADD_OP == Referee::UIFigureOp::UI_OP_ADD)
        {
          Referee::UICharacter char_fig{};
          referee_->FillCharacter(char_fig, "WM", ADD_OP, 1,
                                  Referee::UIColor::UI_COLOR_CYAN, 27, 2, 1345, 764,
                                  mode_str);
          referee_->SendUICharacter(id, client, char_fig);
          last_ui_mode_ = current_mode_;
        }
        break;
      }
      case 1:
      {
        if (ADD_OP == Referee::UIFigureOp::UI_OP_ADD)
        {
          Referee::UIFigure line_fig{};
          referee_->FillLine(line_fig, "WSL", ADD_OP, 1, Referee::UIColor::UI_COLOR_CYAN,
                             2, 0, 0, 400, 1000);
          referee_->SendUIFigure(id, client, line_fig);
        }
        break;
      }
      case 3:
      {
        if (ADD_OP == Referee::UIFigureOp::UI_OP_ADD)
        {
          Referee::UIFigure line_fig{};
          referee_->FillLine(line_fig, "WSR", ADD_OP, 1, Referee::UIColor::UI_COLOR_CYAN,
                             2, 1920, 0, 1520, 1000);
          referee_->SendUIFigure(id, client, line_fig);
        }
        break;
      }
      case 2:
      {
        Referee::UIFigure spfig{};
        Referee::UIColor cap_state;

        float cap_energy = power_control_->GetCapEnergy();
        if (cap_energy < 0.35f)
        {
          cap_state = Referee::UIColor::UI_COLOR_ORANGE;
          this->cnt_low++;
        }
        else
        {
          cap_state = Referee::UIColor::UI_COLOR_CYAN;
          this->cnt_high++;
        }
        this->cap_ener = cap_energy;
        referee_->FillLine(spfig, "MSP", ADD_OP, 1, cap_state, 20, 50, 550,
                           static_cast<uint16_t>(100 + 450 * cap_energy), 550);
        referee_->SendUIFigure(id, client, spfig);
        this->ui_dyn_step_ = (this->ui_dyn_step_ + 1) % 20;
      }
      default:
        break;
    }

    this->ui_step_ = (this->ui_step_ + 1) % 4;
  }

 private:
  const ChassisParam PARAM;

  static constexpr float GRAVITY_ACCELERATION = 9.80665f;
  /* 沿用舵轮前馈参数，保持现有 ChassisParam/YAML 构造接口。 */
  static constexpr uint32_t GRAVITY_FF_TIMEOUT_MS = 50;
  static constexpr float GRAVITY_FF_LPF_HZ = 10.0f;
  static constexpr float GRAVITY_FF_ON_RAD = 3.0f * static_cast<float>(M_PI) / 180.0f;
  static constexpr float GRAVITY_FF_OFF_RAD = 2.0f * static_cast<float>(M_PI) / 180.0f;
  static constexpr float GRAVITY_FF_FRICTION = 0.7f;
  /* +1：yaw 俯视逆时针、pitch 抬头为正，沿用舵轮约定，上车核对。 */
  static constexpr float GRAVITY_FF_YAW_SIGN = 1.0f;
  static constexpr float GRAVITY_FF_PITCH_SIGN = 1.0f;
  static constexpr float FRICTION_OMEGA_DEADBAND = 0.1f;  ///< 输出轴角速度死区 (rad/s)

  struct GravityFeedforwardDebug
  {
    float g_body[3] = {0.0f, 0.0f, 0.0f};       /* 底盘系重力 (m/s^2) */
    float a_g[2] = {0.0f, 0.0f};                /* 限幅后补偿加速度 (m/s^2) */
    float tilt_deg = 0.0f;                      /* 底盘倾角 (deg) */
    float torque[4] = {0.0f, 0.0f, 0.0f, 0.0f}; /* 各轮补偿力矩 (N*m) */
    bool input_valid = false;
    bool active = false;
    uint32_t power_limited_count = 0;
  };

  bool feedback_valid_ = false;
  float gravity_torque_ff_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::EulerAngle<float> gravity_euler_{};
  float gravity_pitch_motor_angle_ = 0.0f;
  LibXR::MillisecondTimestamp gravity_euler_rx_time_ = 0;
  LibXR::MillisecondTimestamp gravity_yaw_rx_time_ = 0;
  LibXR::MillisecondTimestamp gravity_pitch_rx_time_ = 0;
  bool gravity_euler_received_ = false;
  bool gravity_yaw_received_ = false;
  bool gravity_pitch_received_ = false;
  float gravity_body_filtered_[3] = {0.0f, 0.0f, 0.0f};
  bool gravity_filter_initialized_ = false;
  bool gravity_ff_engaged_ = false;
  GravityFeedforwardDebug gravity_ff_debug_{};

  float target_motor_omega_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_force_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_current_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float friction_torque_ff_[4]{0.0f, 0.0f, 0.0f, 0.0f};

  float output_[4]{0.0f, 0.0f, 0.0f, 0.0f};

  float now_vx_ = 0.0f;
  float now_vy_ = 0.0f;
  float now_omega_ = 0.0f;

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;
  float rotor_dynamic_scale_ = 1.0f; /* 功率相关动态缩放 */

  float current_yaw_ = 0.0f;
  float yawmotor_zero_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;

  Motor* motor_wheel_[4]{motor_wheel_0_, motor_wheel_1_, motor_wheel_2_, motor_wheel_3_};
  Motor::Feedback motor_feedback_[4]{};
  Motor::MotorCmd motor_cmd_[4]{};
  MotorData motor_data_{};

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

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  PowerControl* power_control_;
  PowerControlData power_control_data_;

  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  Referee::ChassisPack referee_chassis_pack_{};

  ChassisTopicNames topic_names_{};
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  ChassisMode chassis_event_ = ChassisMode::RELAX;
  Referee* referee_;

  uint16_t ui_step_ = 0;
  uint16_t ui_dyn_step_ = 0;
  ChassisMode current_mode_ = ChassisMode::RELAX;
  ChassisMode last_ui_mode_ = ChassisMode::RELAX;
  LibXR::MillisecondTimestamp ui_update_time_ = 0;

  uint16_t cnt_high = 0;
  uint16_t cnt_low = 0;
  float cap_ener = 0.0f;
};
