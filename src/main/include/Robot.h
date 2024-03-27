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


#include "AbsoluteEncoder.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  double kElevatorP = 0.0;
  double kElevatorI = 0.0;
  double kElevatorD = 0.0;
  double kElevatorS = 0.0;
  double kElevatorG = 0.0;
  double kElevatorV = 0.0;
  double kElevatorA = 0.0;

    // constexpr double kArmP = 0.002;
    // constexpr double kArmI = 0.0;
    // constexpr double kArmD = 0.0;

    // constexpr double kArmS = 0.0;
    // constexpr double kArmG = 0.15;
    // constexpr double kArmWristG = 0.07;
    // constexpr double kArmV = 1.1;
    // constexpr double kArmA = 0.0;


  double kArmP = 0.006;
  double kArmI = 0.0;
  double kArmD = 0.0;

  double kArmS = 0.0;
  double kArmG = 0.3;
  double kArmWristG = 0.01;
  double kArmV = 1.1;
  double kArmA = 0.0;


    // constexpr double kWristP = 1.0;
    // constexpr double kWristI = 0.0;
    // constexpr double kWristD = 0.0;

    // constexpr double kWristS = 0.0;
    // constexpr double kWristG = 0.0115;
    // constexpr double kWristV = 0.65;
    // constexpr double kWristA = 0.0;

  double kWristP = 1.0;
  double kWristI = 0.0;
  double kWristD = 0.0;

  double kWristS = 0.0;
  double kWristG = 0.0115;
  double kWristV = 0.65;
  double kWristA = 0.0;

 
  double kShooterP = 0.01;
  double kShooterI = 0.0;
  double kShooterD = 0.0;
  double kShooterS = 0.0;
  double kShooterG = 0.1;
  double kShooterV = 8.0;
  double kShooterA = 0.0;



  frc::PIDController m_elevatorPID{kElevatorP, kElevatorI, kElevatorD};
  frc::ElevatorFeedforward m_elevatorFeedforward{units::volt_t{kElevatorS}, units::volt_t{kElevatorG}, 
                                        units::unit_t<frc::ElevatorFeedforward::kv_unit> {kElevatorV}, 
                                        units::unit_t<frc::ElevatorFeedforward::ka_unit> {kElevatorA}};
  
  frc::TrapezoidProfile<units::meters> m_elevatorProfile{{1_mps, 1_mps_sq}};
  frc::TrapezoidProfile<units::meters>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::meters>::State m_elevatorSetpoint;

  units::meter_t m_elevatorHeightGoal = 0_m;
  units::meter_t m_elevatorPosition;

  bool offsetsSet = false;


  ctre::phoenix6::hardware::TalonFX m_wristMotor{22};
  ctre::phoenix6::hardware::CANcoder m_wristEncoder{24};
  ctre::phoenix6::controls::MotionMagicDutyCycle m_wristPositionDC{0_deg};
  ctre::phoenix6::StatusSignal<units::turn_t> wristPos = m_wristMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> wristVel = m_wristMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<double> wristPosReference = m_wristMotor.GetClosedLoopReference();
  ctre::phoenix6::StatusSignal<double> wristVelReference = m_wristMotor.GetClosedLoopReferenceSlope();

 // frc::PIDController m_wristPID{ kWristP, kWristI, kWristD };
 // frc::ArmFeedforward m_wristFeedforward{ units::volt_t{ kWristS }, units::volt_t{ kWristG }, 
 //                                       units::unit_t<frc::ArmFeedforward::kv_unit> { kWristV }, 
 //                                       units::unit_t<frc::ArmFeedforward::ka_unit> { kWristA } };

 // frc::TrapezoidProfile<units::degrees> m_wristProfile{{360_deg_per_s, 360_deg_per_s_sq}};
  // frc::TrapezoidProfile<units::degrees>::State m_wristSetpoint;
  frc::TrapezoidProfile<units::degrees>::State m_wristGoal;

  units::degree_t m_wristAngle;

  ctre::phoenix6::hardware::TalonFX m_armMotor{21};
  ctre::phoenix6::hardware::CANcoder m_armEncoder{23};
  ctre::phoenix6::controls::MotionMagicDutyCycle m_armPositionDC{0_deg};
  ctre::phoenix6::StatusSignal<units::turn_t> armPos = m_armMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> armVel = m_armMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<double> armPosReference = m_armMotor.GetClosedLoopReference();
  ctre::phoenix6::StatusSignal<double> armVelReference = m_armMotor.GetClosedLoopReferenceSlope();

  frc::PIDController m_armPID{kArmP, kArmI, kArmD};
  frc::ArmFeedforward *m_armFeedforward;
  
  frc::TrapezoidProfile<units::degrees> m_armProfile{{360_deg_per_s, 360_deg_per_s_sq}};
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  frc::TrapezoidProfile<units::degrees>::State m_armSetpoint{};

  units::degree_t m_armAngle;
  units::degree_t phi;
 
  double ArmWristG;


  // rev::CANSparkFlex m_topShooterMotor{14, rev::CANSparkFlex::MotorType::kBrushless};
  // rev::CANSparkFlex m_bottomShooterMotor{15, rev::CANSparkFlex::MotorType::kBrushless};

  // rev::CANSparkMax m_shooterMotor{19, rev::CANSparkMax::MotorType::kBrushless};
  // ctre::phoenix6::hardware::CANcoder m_shooterEncoder{20};

  // rev::SparkMaxRelativeEncoder m_shooterVel = m_shooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  frc::PIDController m_shooterPID{kShooterP, kShooterI, kShooterD};
  frc::ArmFeedforward m_shooterFeedforward{units::volt_t{kShooterS}, units::volt_t{kShooterG}, 
                                        units::unit_t<frc::ArmFeedforward::kv_unit> {kShooterV}, 
                                        units::unit_t<frc::ArmFeedforward::ka_unit> {kShooterA}};
  
  frc::TrapezoidProfile<units::degrees> m_shooterProfile{{360_deg_per_s, 360_deg_per_s_sq}};
  frc::TrapezoidProfile<units::degrees>::State m_shooterGoal;
  frc::TrapezoidProfile<units::degrees>::State m_shooterSetpoint{};

  units::degree_t m_shooterAngleGoal;
  units::degree_t m_shooterPosition;


  frc::XboxController m_xbox{1};
};

class TuningParameters {
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

