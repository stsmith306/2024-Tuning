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
#include "AbsoluteEncoder.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void TeleopPeriodic() override;

 private:
  // double kElevatorP = 0.0;
  // double kElevatorI = 0.0;
  // double kElevatorD = 0.0;
  // double kElevatorS = 0.0;
  // double kElevatorG = 0.0;
  // double kElevatorV = 0.0;
  // double kElevatorA = 0.0;

    // constexpr double kArmP = 0.002;
    // constexpr double kArmI = 0.0;
    // constexpr double kArmD = 0.0;

    // constexpr double kArmS = 0.0;
    // constexpr double kArmG = 0.15;
    // constexpr double kArmWristG = 0.07;
    // constexpr double kArmV = 1.1;
    // constexpr double kArmA = 0.0;

  tuning::Parameters kArm = {
    0.006, 0.0, 0.0, 0.0, 0.3, 1.1, 0.0 };
  double kArmWristG = 0.01;

  tuning::ParameterUI armUI{ "ArmSubsystem", "Arm Angle", kArm };

    // constexpr double kWristP = 1.0;
    // constexpr double kWristI = 0.0;
    // constexpr double kWristD = 0.0;

    // constexpr double kWristS = 0.0;
    // constexpr double kWristG = 0.0115;
    // constexpr double kWristV = 0.65;
    // constexpr double kWristA = 0.0;

  tuning::Parameters kWrist = {
    1.0, 0.0, 0.0, 0.0, 0.0115, 0.65, 0.0 };

  tuning::ParameterUI wristUI{ "ArmSubsystem", "Wrist Angle", kWrist };

 
  // double kShooterP = 0.01;
  // double kShooterI = 0.0;
  // double kShooterD = 0.0;
  // double kShooterS = 0.0;
  // double kShooterG = 0.1;
  // double kShooterV = 8.0;
  // double kShooterA = 0.0;

  
  // frc::TrapezoidProfile<units::meters> m_elevatorProfile{{1_mps, 1_mps_sq}};
  // frc::TrapezoidProfile<units::meters>::State m_elevatorGoal;
  // frc::TrapezoidProfile<units::meters>::State m_elevatorSetpoint;

  // units::meter_t m_elevatorHeightGoal = 0_m;
  // units::meter_t m_elevatorPosition;

  // bool offsetsSet = false;


  TalonFXTuner m_wristMotor{ "Wrist", 22, "", kWrist, tuning::Arm, tuning::OnBoard};
  CTRECANCoder m_wristEncoder{ "Wrist Encoder", 24 };

  TalonFXTuner m_armMotor{ "Arm", 21, "", kArm, tuning::Arm, tuning::Software };
  CTRECANCoder m_armEncoder{ "Arm Encoder", 23 };

 // frc::PIDController m_wristPID{ kWristP, kWristI, kWristD };
 // frc::ArmFeedforward m_wristFeedforward{ units::volt_t{ kWristS }, units::volt_t{ kWristG }, 
 //                                       units::unit_t<frc::ArmFeedforward::kv_unit> { kWristV }, 
 //                                       units::unit_t<frc::ArmFeedforward::ka_unit> { kWristA } };

 // frc::TrapezoidProfile<units::degrees> m_wristProfile{{360_deg_per_s, 360_deg_per_s_sq}};
  // frc::TrapezoidProfile<units::degrees>::State m_wristSetpoint;
  frc::TrapezoidProfile<units::degrees>::State m_wristGoal;

  units::degree_t m_wristAngle;
  
  // frc::TrapezoidProfile<units::degrees> m_armProfile{{360_deg_per_s, 360_deg_per_s_sq}};
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  // frc::TrapezoidProfile<units::degrees>::State m_armSetpoint{};

  units::degree_t m_armAngle;
  units::degree_t phi;
 
  double ArmWristG;


  // rev::CANSparkFlex m_topShooterMotor{14, rev::CANSparkFlex::MotorType::kBrushless};
  // rev::CANSparkFlex m_bottomShooterMotor{15, rev::CANSparkFlex::MotorType::kBrushless};

  // rev::CANSparkMax m_shooterMotor{19, rev::CANSparkMax::MotorType::kBrushless};
  // ctre::phoenix6::hardware::CANcoder m_shooterEncoder{20};

  // rev::SparkMaxRelativeEncoder m_shooterVel = m_shooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  // frc::PIDController m_shooterPID{kShooterP, kShooterI, kShooterD};
  // frc::ArmFeedforward m_shooterFeedforward{units::volt_t{kShooterS}, units::volt_t{kShooterG}, 
  //                                       units::unit_t<frc::ArmFeedforward::kv_unit> {kShooterV}, 
  //                                       units::unit_t<frc::ArmFeedforward::ka_unit> {kShooterA}};
  
  // frc::TrapezoidProfile<units::degrees> m_shooterProfile{{360_deg_per_s, 360_deg_per_s_sq}};
  // frc::TrapezoidProfile<units::degrees>::State m_shooterGoal;
  // frc::TrapezoidProfile<units::degrees>::State m_shooterSetpoint{};

  // units::degree_t m_shooterAngleGoal;
  // units::degree_t m_shooterPosition;


  frc::XboxController m_xbox{1};
};

class TuningParameters_old {
  public:
    struct Values {
      double kP;
      double kI;
      double kD;
      double kS;
      double kG;
      double kV;
      double kA;
    };
    static void SetSmartDashboardValues( const std::string_view &name, const Values &v );
    static Values GetSmartDashboardValues( const std::string_view &name );

};

