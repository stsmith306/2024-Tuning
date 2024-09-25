// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

void Robot::RobotInit() {
  // m_shooterMotor.SetInverted(true);
  // m_shooterMotor.EnableVoltageCompensation(12);

  // m_topShooterMotor.SetInverted(true);
  // m_bottomShooterMotor.SetInverted(true);
  // m_bottomShooterMotor.Follow(m_topShooterMotor);

  m_armEncoder.SensorDirection( true );
  m_wristEncoder.SensorDirection( true );

  m_armMotor.SetInverted( true );
  m_armMotor.SetMotionProfile( armUI.GetMotionProfile() );

  m_wristMotor.LinkCANCoder( m_wristEncoder.GetChannel(), 36 * 42.0 / 38.0 /* Wrist Gear Ratio */ );
  m_wristMotor.SetMotionProfile( wristUI.GetMotionProfile() );

  ArmWristG = kArmWristG;
  frc::SmartDashboard::PutNumber( "ArmWristG", ArmWristG ); 

  frc::SmartDashboard::PutBoolean("Config Gains", false);
  frc::SmartDashboard::PutBoolean("Set Arm Zeros", false);

}

void Robot::RobotPeriodic() {

  m_wristAngle = m_wristEncoder.GetPosition();
  phi = m_armEncoder.GetPosition();

  m_armAngle = m_wristAngle - phi;

  frc::SmartDashboard::PutNumber("Phi", phi.value());

    // The extra kG needd for the arm based on the wrist angle.
  double armExtraFeedforward = ArmWristG * units::math::cos(m_wristAngle).value();

  m_armMotor.Update( m_armAngle, armExtraFeedforward );
  m_wristMotor.Update( m_wristEncoder.GetPosition() );

  bool configGains = frc::SmartDashboard::GetBoolean("Config Gains", false);
  if (configGains) {
    m_armMotor.SetParameters( armUI.GetParameters() );
    m_armMotor.SetMotionProfile( armUI.GetMotionProfile() );

    m_wristMotor.SetParameters( wristUI.GetParameters() );
    m_wristMotor.SetMotionProfile( wristUI.GetMotionProfile() );

    ArmWristG = frc::SmartDashboard::GetNumber( "ArmWristG", 0.0 ); 

    frc::SmartDashboard::PutBoolean("Config Gains", false);
    fmt::print(" Gain Values written...\n" );
  } 

  bool setZeros = frc::SmartDashboard::GetBoolean("Set Arm Zeros", false);
  if (setZeros) {
    m_armEncoder.SetOffset( 0_deg );
    m_wristEncoder.SetOffset( 0_deg );

    frc::SmartDashboard::PutBoolean("Set Arm Zeros", false);
    fmt::print(" Encoders Zeroed...\n" );
  }
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
    m_armMotor.SetGoal( {-8_deg, 0_deg_per_s} ); 
    m_wristMotor.SetGoal( {-35_deg, 0_deg_per_s} ); 
  } else if (m_xbox.GetBButton()) {
    m_armMotor.SetGoal( {155_deg, 0_deg_per_s} ); 
    m_wristMotor.SetGoal( {40_deg, 0_deg_per_s} ); 
  } else if (m_xbox.GetXButton()) {
    m_armMotor.SetGoal( {0_deg, 0_deg_per_s} ); 
    m_wristMotor.SetGoal( {0_deg, 0_deg_per_s} ); 
  } else if (m_xbox.GetYButton()) {
    m_armMotor.SetGoal( {160_deg, 0_deg_per_s} ); 
    m_wristMotor.SetGoal( {130_deg, 0_deg_per_s} ); 
  }
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif


