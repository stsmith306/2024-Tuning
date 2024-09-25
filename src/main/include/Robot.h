// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/XboxController.h>
#include <units/frequency.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/CANSparkFlex.h>
#include <ctre/phoenix6/TalonFX.hpp>


#include "CTRETuner.h"
#include "CTRECANcoder.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void TeleopPeriodic() override;

 private:

  tuning::Parameters kArm{ 0.006, 0.0, 0.0, 0.0, 0.3, 1.1, 0.0 };
  tuning::AngularMotionProfile kArm_MP{540_deg_per_s, 720_deg_per_s_sq, 0_deg_per_s_cu};
  tuning::AngularTuningUI armUI{ "ArmSubsystem", "Arm", kArm, kArm_MP };

  tuning::Parameters kWrist{ 1.0, 0.0, 0.0, 0.005, 0.018, 0.38, 0.0 };
  double kArmWristG = 0.01;
  tuning::AngularMotionProfile kWrist_MP{540_deg_per_s, 1080_deg_per_s_sq, 2000_deg_per_s_cu};
  tuning::AngularTuningUI wristUI{ "ArmSubsystem", "Wrist", kWrist, kWrist_MP };

  AngularTalonFXTuner m_wristMotor{ "Wrist", 22, "", kWrist, tuning::Arm, tuning::OnBoard};

  CTRECANcoder m_wristEncoder{ "WristOffset", 24 };

  AngularTalonFXTuner m_armMotor{ "Arm", 21, "", kArm, tuning::Arm, tuning::Software };
  CTRECANcoder m_armEncoder{ "ArmOffset", 23 };

  units::degree_t m_wristAngle;
  
  units::degree_t m_armAngle;
  units::degree_t phi;
 
  double ArmWristG;

  frc::XboxController m_xbox{1};
};

