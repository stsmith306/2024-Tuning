// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

void Robot::RobotInit() {
  // m_shooterMotor.SetInverted(true);
  // m_shooterMotor.EnableVoltageCompensation(12);

  // ctre::phoenix6::configs::CANcoderConfiguration absoluteEncoderConfigs{};
  // absoluteEncoderConfigs.MagnetSensor.MagnetOffset = 0.301;
  // m_shooterEncoder.GetConfigurator().Apply(absoluteEncoderConfigs, 50_ms);

  // m_topShooterMotor.SetInverted(true);
  // m_bottomShooterMotor.SetInverted(true);
  // m_bottomShooterMotor.Follow(m_topShooterMotor);

  ctre::phoenix6::configs::CANcoderConfiguration wristAbsoluteEncoderConfigs{};
  wristAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = true;
  wristAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = 0.145;
  m_wristEncoder.GetConfigurator().Apply(wristAbsoluteEncoderConfigs, 50_ms);

  ctre::phoenix6::configs::CANcoderConfiguration armAbsoluteEncoderConfigs{};
  armAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = true;
  armAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = 0.081;
  m_armEncoder.GetConfigurator().Apply(armAbsoluteEncoderConfigs, 50_ms);

  m_armMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

  ctre::phoenix6::configs::TalonFXConfiguration armConfigs{};
  armConfigs.Slot0.kV = kArmV;
  armConfigs.Slot0.kP = kArmP;
  armConfigs.Slot0.kI = kArmI;
  armConfigs.Slot0.kD = kArmD;
  armConfigs.Feedback.FeedbackRemoteSensorID = m_armEncoder.GetDeviceID();
  armConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  armConfigs.MotionMagic.MotionMagicAcceleration = 1;
  armConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.1;
  armConfigs.MotorOutput.Inverted = true;
  m_armMotor.GetConfigurator().Apply(armConfigs, 50_ms);


  m_wristMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

  ctre::phoenix6::configs::TalonFXConfiguration wristConfigs{};
  wristConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
  wristConfigs.Slot0.kG = kWristG;
  wristConfigs.Slot0.kV = kWristV;
  wristConfigs.Slot0.kP = kWristP;
  wristConfigs.Slot0.kI = kWristI;
  wristConfigs.Slot0.kD = kWristD;
  wristConfigs.Feedback.FeedbackRemoteSensorID = m_wristEncoder.GetDeviceID();
  wristConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  wristConfigs.MotionMagic.MotionMagicAcceleration = 1;
  wristConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.5;
  m_wristMotor.GetConfigurator().Apply(wristConfigs, 50_ms);

  auto status = wristPosReference.SetUpdateFrequency(50_Hz);
  if( status != ctre::phoenix::StatusCode::OK ) {
      fmt::print( "Error setting wrist pos update Frequency!\n    {}\n", status.GetDescription() );
  }
  status = wristVelReference.SetUpdateFrequency(50_Hz);
  if( status != ctre::phoenix::StatusCode::OK ) {
      fmt::print( "Error setting wrist vel update Frequency!\n    {}\n", status.GetDescription() );
  }

  armPosReference.SetUpdateFrequency(50_Hz);
  armVelReference.SetUpdateFrequency(50_Hz);

  fmt::print( "Pos and Velocity update rates are : {} and {}\n", wristPosReference.GetAppliedUpdateFrequency(), wristVelReference.GetAppliedUpdateFrequency() );

  frc::SmartDashboard::PutNumber( "Elevator P", kElevatorP );
  frc::SmartDashboard::PutNumber( "Elevator I", kElevatorI );
  frc::SmartDashboard::PutNumber( "Elevator D", kElevatorD );
  frc::SmartDashboard::PutNumber( "Elevator S", kElevatorS );
  frc::SmartDashboard::PutNumber( "Elevator G", kElevatorG );
  frc::SmartDashboard::PutNumber( "Elevator V", kElevatorV );
  frc::SmartDashboard::PutNumber( "Elevator A", kElevatorA );

  frc::SmartDashboard::PutNumber( "Arm P", kArmP );
  frc::SmartDashboard::PutNumber( "Arm I", kArmI );
  frc::SmartDashboard::PutNumber( "Arm D", kArmD );
  frc::SmartDashboard::PutNumber( "Arm S", kArmS );
  frc::SmartDashboard::PutNumber( "Arm G", kArmG );
  frc::SmartDashboard::PutNumber( "Arm V", kArmV );
  frc::SmartDashboard::PutNumber( "Arm A", kArmA );

  frc::SmartDashboard::PutNumber( "Wrist P", kWristP );
  frc::SmartDashboard::PutNumber( "Wrist I", kWristI );
  frc::SmartDashboard::PutNumber( "Wrist D", kWristD );
  frc::SmartDashboard::PutNumber( "Wrist S", kWristS );
  frc::SmartDashboard::PutNumber( "Wrist G", kWristG );
  frc::SmartDashboard::PutNumber( "Wrist V", kWristV );
  frc::SmartDashboard::PutNumber( "Wrist A", kWristA );

  frc::SmartDashboard::PutNumber( "Shooter P", kShooterP );
  frc::SmartDashboard::PutNumber( "Shooter I", kShooterI );
  frc::SmartDashboard::PutNumber( "Shooter D", kShooterD );
  frc::SmartDashboard::PutNumber( "Shooter S", kShooterS );
  frc::SmartDashboard::PutNumber( "Shooter G", kShooterG );
  frc::SmartDashboard::PutNumber( "Shooter V", kShooterV );
  frc::SmartDashboard::PutNumber( "Shooter A", kShooterA );

  

}

void Robot::RobotPeriodic() {
  // if(!offsetsSet) {
  //   if(m_wristEncoder.IsConnected() && m_armEncoder.IsConnected()) {
  //     m_wristEncoder.SetPositionOffset();
  //     m_armEncoder.SetPositionOffset();
  //     offsetsSet = true;
  //   }
  // }
  wristPos.Refresh();
  wristVel.Refresh();
  wristPosReference.Refresh();
  wristVelReference.Refresh();

  armPos.Refresh();
  armVel.Refresh();
  armPosReference.Refresh();
  armVelReference.Refresh();

  // m_shooterPosition = m_shooterEncoder.GetPosition().GetValueAsDouble() * 360_deg;
  alpha = m_wristEncoder.GetPosition().GetValueAsDouble() * 360_deg;
  phi = m_armEncoder.GetPosition().GetValueAsDouble() * 360_deg;

  m_armPosition = alpha - phi;

  frc::SmartDashboard::PutNumber("Alpha", alpha.value());
  frc::SmartDashboard::PutNumber("Phi", phi.value());
  frc::SmartDashboard::PutNumber("Arm Pos", m_armPosition.value());

  // frc::SmartDashboard::PutNumber("Wrist Raw Position", m_wristEncoder.GetRawPosition());
  // frc::SmartDashboard::PutNumber("Arm Raw Position", m_armEncoder.GetRawPosition());

  frc::SmartDashboard::PutNumber("Arm Encoder Position", m_armMotor.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Arm Encoder Velocity", m_armMotor.GetVelocity().GetValueAsDouble());


  // double elevatorP = frc::SmartDashboard::GetNumber( "Elevator P", 0.0 );
  // double elevatorI = frc::SmartDashboard::GetNumber( "Elevator I", 0.0 );
  // double elevatorD = frc::SmartDashboard::GetNumber( "Elevator D", 0.0 );
  // double elevatorG = frc::SmartDashboard::GetNumber( "Elevator G", 0.0 );
  // double elevatorV = frc::SmartDashboard::GetNumber( "Elevator V", 0.0 );
  // double elevatorS = frc::SmartDashboard::GetNumber( "Elevator S", 0.0 );
  // double elevatorA = frc::SmartDashboard::GetNumber( "Elevator A", 0.0 );

  double armP = frc::SmartDashboard::GetNumber( "Arm P", 0.0 );
  double armI = frc::SmartDashboard::GetNumber( "Arm I", 0.0 );
  double armD = frc::SmartDashboard::GetNumber( "Arm D", 0.0 );
  double armG = frc::SmartDashboard::GetNumber( "Arm G", 0.0 );
  double armV = frc::SmartDashboard::GetNumber( "Arm V", 0.0 );
  double armS = frc::SmartDashboard::GetNumber( "Arm S", 0.0 );
  double armA = frc::SmartDashboard::GetNumber( "Arm A", 0.0 );

  double wristP = frc::SmartDashboard::GetNumber( "Wrist P", 0.0 );
  double wristI = frc::SmartDashboard::GetNumber( "Wrist I", 0.0 );
  double wristD = frc::SmartDashboard::GetNumber( "Wrist D", 0.0 );
  double wristG = frc::SmartDashboard::GetNumber( "Wrist G", 0.0 );
  double wristV = frc::SmartDashboard::GetNumber( "Wrist V", 0.0 );
  double wristS = frc::SmartDashboard::GetNumber( "Wrist S", 0.0 );
  double wristA = frc::SmartDashboard::GetNumber( "Wrist A", 0.0 );

  bool configGains = frc::SmartDashboard::GetBoolean("Config Gains", false);

  if (configGains) {
    ctre::phoenix6::configs::TalonFXConfiguration armPIDConfigs{};
    armPIDConfigs.Slot0.kV = armV;
    armPIDConfigs.Slot0.kP = armP;
    armPIDConfigs.Slot0.kI = armI;
    armPIDConfigs.Slot0.kD = armD;
    m_armMotor.GetConfigurator().Apply(armPIDConfigs);
    frc::SmartDashboard::PutBoolean("Config Gains", false);
  } else {
    frc::SmartDashboard::PutBoolean("Config Gains", false);
  }

  // double shooterP = frc::SmartDashboard::GetNumber( "Shooter P", 0.0 );
  // double shooterI = frc::SmartDashboard::GetNumber( "Shooter I", 0.0 );
  // double shooterD = frc::SmartDashboard::GetNumber( "Shooter D", 0.0 );
  // double shooterG = frc::SmartDashboard::GetNumber( "Shooter G", 0.0 );
  // double shooterV = frc::SmartDashboard::GetNumber( "Shooter V", 0.0 );
  // double shooterS = frc::SmartDashboard::GetNumber( "Shooter S", 0.0 );
  // double shooterA = frc::SmartDashboard::GetNumber( "Shooter A", 0.0 );

  // if ( elevatorP != kElevatorP ) { kElevatorP = elevatorP; m_elevatorPID.SetP( kElevatorP ); }
  // if ( elevatorI != kElevatorI ) { kElevatorI = elevatorI; m_elevatorPID.SetI( kElevatorI ); }
  // if ( elevatorD != kElevatorD ) { kElevatorD = elevatorD; m_elevatorPID.SetD( kElevatorD ); }

  // if ( armP != kArmP ) { kArmP = armP; m_armPID.SetP( kArmP ); }
  // if ( armI != kArmI ) { kArmI = armI; m_armPID.SetI( kArmI ); }
  // if ( armD != kArmD ) { kArmD = armD; m_armPID.SetD( kArmD ); }

  // if ( wristP != kWristP ) { kWristP = wristP; m_wristPID.SetP( kWristP ); }
  // if ( wristI != kWristI ) { kWristI = wristI; m_wristPID.SetI( kWristI ); }
  // if ( wristD != kWristD ) { kWristD = wristD; m_wristPID.SetD( kWristD ); }

  // if ( shooterP != kShooterP ) { kShooterP = shooterP; m_shooterPID.SetP( kShooterP ); }
  // if ( shooterI != kShooterI ) { kShooterI = shooterI; m_shooterPID.SetI( kShooterI ); }
  // if ( shooterD != kShooterD ) { kShooterD = shooterD; m_shooterPID.SetD( kShooterD ); }
  
  

  // frc::SmartDashboard::PutNumber("Shooter Position", m_shooterPosition.value());




  // if ( frc::DriverStation::IsDisabled() ) {
  //   m_shooterSetpoint.position = m_shooterPosition;
  //   m_shooterSetpoint.velocity = 0_deg_per_s;
  //   m_shooterAngleGoal = m_shooterPosition;
  // }

  if ( frc::DriverStation::IsDisabled() ) {
    m_armSetpoint.position = m_armPosition;
    m_armSetpoint.velocity = 0_deg_per_s;
    m_armAngleGoal = m_armPosition;

    m_wristSetpoint.position = alpha;
    m_wristSetpoint.velocity = 0_deg_per_s;
    m_wristAngleGoal = m_wristPosition;

    thetaSetpoint = m_armPosition;
    alphaSetpoint = alpha;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  
  // if(m_xbox.GetAButton()){
  //   m_shooterGoal = {30_deg, 0_deg_per_s};
  // } else if (m_xbox.GetBButton()) {
  //   m_shooterGoal = {60_deg, 0_deg_per_s}; 
  // }

  // if (m_xbox.GetXButton()) {
  //   m_topShooterMotor.Set(0.25);
  // } else {
  //   m_topShooterMotor.Set(0.0);
  // }

  // m_shooterSetpoint = m_shooterProfile.Calculate(20_ms, m_shooterSetpoint, m_shooterGoal);

  // frc::SmartDashboard::PutNumber("Shooter Setpoint Position", m_shooterSetpoint.position.value());
  // frc::SmartDashboard::PutNumber("Shooter Setpoint Velocity", m_shooterSetpoint.velocity.value());

  // double shooterOutput = m_shooterPID.Calculate(m_shooterPosition.value(), m_shooterSetpoint.position.value());
  // double shooterFFOutput = m_shooterFeedforward.Calculate(m_shooterSetpoint.position, m_shooterSetpoint.velocity).value();

  // m_shooterMotor.Set(shooterOutput + shooterFFOutput / 12);

  // frc::SmartDashboard::PutNumber("Shooter Velocity", m_shooterVel.GetVelocity() / 48.0 / 10.0 * 24.0 / 36.0 * 360.0 / 60.0 );

  if(m_xbox.GetAButton()){
    m_armGoal = {10_deg, 0_deg_per_s};
    m_wristGoal = {-60_deg, 0_deg_per_s};
  } else if (m_xbox.GetBButton()) {
    m_armGoal = {120_deg, 0_deg_per_s}; 
    m_wristGoal = {0_deg, 0_deg_per_s};
  } else if (m_xbox.GetXButton()) {
    m_armGoal = {135_deg, 0_deg_per_s}; 
    m_wristGoal = {150_deg, 0_deg_per_s};
  }
  
    
  m_wristSetpoint = m_wristProfile.Calculate(20_ms, m_wristSetpoint, m_wristGoal);
  m_armSetpoint = m_armProfile.Calculate(20_ms, m_armSetpoint, m_armGoal);

  frc::SmartDashboard::PutNumber("Wrist Setpoint Position", m_wristSetpoint.position.value());
  frc::SmartDashboard::PutNumber("Wrist Setpoint Velocity", m_wristSetpoint.velocity.value());

  frc::SmartDashboard::PutNumber("Arm Setpoint Position", m_armSetpoint.position.value());
  frc::SmartDashboard::PutNumber("Arm Setpoint Velocity", m_armSetpoint.velocity.value());


  double wristOutput = m_wristPID.Calculate(alpha.value(), m_wristSetpoint.position.value());
  double wristFeedforwardOut = m_wristFeedforward.Calculate(alpha, m_wristSetpoint.velocity).value();

  // m_wristMotor.Set(wristOutput + wristFeedforwardOut / 12.0);
  if(m_xbox.GetAButton()) {
    alphaSetpoint = -90_deg;
  } else if (m_xbox.GetBButton()) {
    alphaSetpoint = 0_deg;
  }
  m_wristMotor.SetControl(m_wristPositionDC.WithPosition(alphaSetpoint));

  frc::SmartDashboard::PutNumber("Wrist Position", wristPos.GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Wrist Velocity", wristVel.GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Wrist Motion Magic Pos", wristPosReference.GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Wrist Motion Magic Vel", wristVelReference.GetValueAsDouble());
  

  // double armOutput = m_armPID.Calculate(m_armPosition.value(), m_armSetpoint.position.value());
  // double armFeedforwardOut = m_armFeedforward.Calculate(m_armSetpoint.position, m_armSetpoint.velocity).value() + kArmWristG * units::math::cos(alpha).value();

  // frc::SmartDashboard::PutNumber("Arm PID Out", armOutput);
  // frc::SmartDashboard::PutNumber("Arm Feedforward Out", armFeedforwardOut);

  //m_armMotor.Set(armOutput + armFeedforwardOut / 12.0);

  double armFeedforwardOut = kArmG * units::math::cos(m_armPosition) + kArmWristG * units::math::cos(alpha);
  frc::SmartDashboard::PutNumber("Arm Feedforward Out", armFeedforwardOut);
  if(m_xbox.GetXButton()) {
    thetaSetpoint = 0_deg;
  } else if(m_xbox.GetYButton()) {
    thetaSetpoint = 45_deg;
  }
  m_armMotor.SetControl(m_armPositionDC.WithPosition(alpha + thetaSetpoint).WithFeedForward(armFeedforwardOut));

  frc::SmartDashboard::PutNumber("Arm Position", armPos.GetValueAsDouble() * 360.0);
  frc::SmartDashboard::PutNumber("Arm Velocity", armVel.GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Arm Motion Magic Pos", armPosReference.GetValueAsDouble() * 360.0);
  frc::SmartDashboard::PutNumber("Arm Motion Magic Vel", armVelReference.GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Phi Setpoint", (alpha + thetaSetpoint).value() );
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
