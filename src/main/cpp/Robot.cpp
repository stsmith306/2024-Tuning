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
  // constexpr double kShooterAbsoluteOffset = -0.068;
  // constexpr double kWristAbsoluteOffset = 0.314;
  // constexpr double kArmAbsoluteOffset = -0.243;

  m_armEncoder.SensorDirection( true );
  m_wristEncoder.SensorDirection( true );

  m_armMotor.SetInverted( true );
  m_armMotor.SetMotionProfile( armMPUI.m_prof );

  m_wristMotor.LinkCANCoder( m_wristEncoder.GetChannel() );
  m_wristMotor.SetMotionProfile( wristMPUI.m_prof );


  // TuningParameters::Values pid_vals;
  // // pid_vals = TuningParameters::Values{ kElevatorP, kElevatorI, kElevatorD, 
  // //                                    kElevatorS, kElevatorG, kElevatorV, kElevatorA };
  // // TuningParameters::SetSmartDashboardValues( "Elevator", pid_vals );

  armUI.PutNTValues();
  armMPUI.PutNTValues();
  wristUI.PutNTValues();
  wristMPUI.PutNTValues();

  ArmWristG = kArmWristG;
  frc::SmartDashboard::PutNumber( "ArmWristG", ArmWristG ); 

  // frc::SmartDashboard::PutNumber( "Shooter P", kShooterP );
  // frc::SmartDashboard::PutNumber( "Shooter I", kShooterI );
  // frc::SmartDashboard::PutNumber( "Shooter D", kShooterD );
  // frc::SmartDashboard::PutNumber( "Shooter S", kShooterS );
  // frc::SmartDashboard::PutNumber( "Shooter G", kShooterG );
  // frc::SmartDashboard::PutNumber( "Shooter V", kShooterV );
  // frc::SmartDashboard::PutNumber( "Shooter A", kShooterA );

  frc::SmartDashboard::PutBoolean("Config Gains", false);

}

void Robot::RobotPeriodic() {

  // m_shooterPosition = m_shooterEncoder.GetPosition().GetValueAsDouble() * 360_deg;
  m_wristAngle = m_wristEncoder.GetPosition();
  phi = m_armEncoder.GetPosition();

  m_armAngle = m_wristAngle - phi;

  // frc::SmartDashboard::PutNumber("Wrist Angle", m_wristAngle.value());
  frc::SmartDashboard::PutNumber("Phi", phi.value());
  // frc::SmartDashboard::PutNumber("Arm Angle", m_armAngle.value());

    // The extra kG needd for the arm based on the wrist angle.
  double armExtraFeedforward = ArmWristG * units::math::cos(m_wristAngle).value();

  m_armMotor.Update( m_armAngle, armExtraFeedforward );
  m_wristMotor.Update();


  // frc::SmartDashboard::PutNumber("Wrist Raw Position", m_wristEncoder.GetRawPosition());
  // frc::SmartDashboard::PutNumber("Arm Raw Position", m_armEncoder.GetRawPosition());

  // double elevatorP = frc::SmartDashboard::GetNumber( "Elevator P", 0.0 );
  // double elevatorI = frc::SmartDashboard::GetNumber( "Elevator I", 0.0 );
  // double elevatorD = frc::SmartDashboard::GetNumber( "Elevator D", 0.0 );
  // double elevatorG = frc::SmartDashboard::GetNumber( "Elevator G", 0.0 );
  // double elevatorV = frc::SmartDashboard::GetNumber( "Elevator V", 0.0 );
  // double elevatorS = frc::SmartDashboard::GetNumber( "Elevator S", 0.0 );
  // double elevatorA = frc::SmartDashboard::GetNumber( "Elevator A", 0.0 );

  bool configGains = frc::SmartDashboard::GetBoolean("Config Gains", false);

  if (configGains) {

    m_armMotor.SetParameters( armUI.GetNTValues() );
    m_armMotor.SetMotionProfile( armMPUI.GetNTValues() );

    m_wristMotor.SetParameters( wristUI.GetNTValues() );
    m_wristMotor.SetMotionProfile( wristMPUI.GetNTValues() );

    ArmWristG = frc::SmartDashboard::GetNumber( "ArmWristG", 0.0 ); 

    frc::SmartDashboard::PutBoolean("Config Gains", false);
    fmt::print(" Gain Values written...\n" );
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

  // if ( shooterP != kShooterP ) { kShooterP = shooterP; m_shooterPID.SetP( kShooterP ); }
  // if ( shooterI != kShooterI ) { kShooterI = shooterI; m_shooterPID.SetI( kShooterI ); }
  // if ( shooterD != kShooterD ) { kShooterD = shooterD; m_shooterPID.SetD( kShooterD ); }
  
  

  // frc::SmartDashboard::PutNumber("Shooter Position", m_shooterPosition.value());




  // if ( frc::DriverStation::IsDisabled() ) {
  //   m_shooterSetpoint.position = m_shooterPosition;
  //   m_shooterSetpoint.velocity = 0_deg_per_s;
  //   m_shooterAngleGoal = m_shooterPosition;
  // }

}


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
    m_armGoal = {-8_deg, 0_deg_per_s}; 
    m_wristGoal = {-35_deg, 0_deg_per_s};
  } else if (m_xbox.GetBButton()) {
    m_armGoal = {170_deg, 0_deg_per_s}; 
    m_wristGoal = {35_deg, 0_deg_per_s};
  // } else if (m_xbox.GetXButton()) {
  //   m_armGoal = {0_deg, 0_deg_per_s}; 
  } else if (m_xbox.GetYButton()) {
    m_armGoal = {160_deg, 0_deg_per_s}; 
    m_wristGoal = {130_deg, 0_deg_per_s};
  }

  m_armMotor.SetGoal( m_armGoal );
  m_wristMotor.SetGoal( m_wristGoal );
 
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif


