
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Tuning.h"

using namespace tuning;

AngularTuningUI::AngularTuningUI( std::string tab, std::string name, tuning::Parameters p, tuning::AngularMotionProfile prof )
    : m_tab{tab}, m_name{name}, m_params{p}, m_prof{prof} {

}

void AngularTuningUI::Refresh( ) {
  std::string item_name;
  
  item_name = std::string{m_name} + " P";
  frc::SmartDashboard::PutNumber( item_name, m_params.kP );

  item_name = std::string{m_name} + " I";
  frc::SmartDashboard::PutNumber( item_name, m_params.kI );

  item_name = std::string{m_name} + " D";
  frc::SmartDashboard::PutNumber( item_name, m_params.kD );

  item_name = std::string{m_name} + " S";
  frc::SmartDashboard::PutNumber( item_name, m_params.kS );

  item_name = std::string{m_name} + " G";
  frc::SmartDashboard::PutNumber( item_name, m_params.kG );

  item_name = std::string{m_name} + " V";
  frc::SmartDashboard::PutNumber( item_name, m_params.kV );

  item_name = std::string{m_name} + " A";
  frc::SmartDashboard::PutNumber( item_name, m_params.kA );

  item_name = std::string{m_name} + " MaxVelocity";
  frc::SmartDashboard::PutNumber( item_name, m_prof.MaxVelocity.value() );

  item_name = std::string{m_name} + " MaxAccel";
  frc::SmartDashboard::PutNumber( item_name, m_prof.MaxAcceleration.value() );

  item_name = std::string{m_name} + " MaxJerk";
  frc::SmartDashboard::PutNumber( item_name, m_prof.MaxJerk.value() );
}

tuning::Parameters AngularTuningUI::GetParameters( ) {
  std::string item_name;

  item_name = std::string{m_name} + " P";
  m_params.kP = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " I";
  m_params.kI = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " D";
  m_params.kD = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " S";
  m_params.kS = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " G";
  m_params.kG = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " V";
  m_params.kV = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " A";
  m_params.kA = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  return m_params;
}

tuning::AngularMotionProfile AngularTuningUI::GetMotionProfile( ) {
  std::string item_name;

  item_name = std::string{m_name} + " MaxVelocity";
  m_prof.MaxVelocity = frc::SmartDashboard::GetNumber( item_name, 0.0 ) * 1_deg_per_s;

  item_name = std::string{m_name} + " MaxAccel";
  m_prof.MaxAcceleration = frc::SmartDashboard::GetNumber( item_name, 0.0 ) * 1_deg_per_s_sq;

  item_name = std::string{m_name} + " MaxJerk";
  m_prof.MaxJerk = frc::SmartDashboard::GetNumber( item_name, 0.0 ) * 1_deg_per_s_cu;

  return m_prof;
}



AngularMotorTuner::AngularMotorTuner( std::string_view name, tuning::Parameters p, 
                        tuning::MechanismType mech, tuning::ControlType ctrl ) 
    : m_name{name}, m_parameters{p}, m_mech{mech}, m_ctrl{ctrl}, m_softPID{p.kP, p.kI, p.kD} {

    m_armFF = nullptr;
    m_simpleFF = nullptr;
    m_Profile = nullptr;

    if( m_ctrl == tuning::Software ) {
            // Setup the Software feedforward.
        CreateSoftwareFeedForward();

            // Default to some slow values.
        m_Profile = new frc::TrapezoidProfile<units::degrees>({60_deg_per_s, 60_deg_per_s_sq});
    }
}

void AngularMotorTuner::Update( units::degree_t position, double arbFF ) {
    frc::SmartDashboard::PutNumber( m_name + " Position", position.value());
    frc::SmartDashboard::PutNumber( m_name + " Goal Position", m_Goal.position.value());
    
    if ( frc::DriverStation::IsDisabled() ) {
        m_Setpoint.position = position;
        m_Setpoint.velocity = 0_deg_per_s;
        m_Goal = { m_Setpoint.position, 0_deg_per_s}; 
        return;
    }

    if( m_ctrl == tuning::Software ) {
        m_Setpoint = m_Profile->Calculate(20_ms, m_Setpoint, m_Goal);

        double armOutput = m_softPID.Calculate( position.value(), m_Setpoint.position.value() );
        double armFeedforwardOut = m_armFF->Calculate( m_Setpoint.position, m_Setpoint.velocity).value();

        frc::SmartDashboard::PutNumber( m_name + " PID Out", armOutput);
        frc::SmartDashboard::PutNumber( m_name + " Feedforward Out", armFeedforwardOut );
        frc::SmartDashboard::PutNumber( m_name + " Setpoint Position", m_Setpoint.position.value());
        frc::SmartDashboard::PutNumber( m_name + " Setpoint Velocity", m_Setpoint.velocity.value());

        arbFF += armOutput + armFeedforwardOut / 12.0;
    }

    MotorPeriodic( arbFF );
}

void AngularMotorTuner::Update( double arbFF ) {
    Update( GetPosition(), arbFF );
}

units::degree_t AngularMotorTuner::GetPosition() {
    fmt::print( "MotorTuner::GetPosition() for <{}> NOT OVERRIDDEN!  CONTROL WILL NOT WORK!!!!\n", m_name );
    return 0_deg;
}

void AngularMotorTuner::SetGoal( frc::TrapezoidProfile<units::degrees>::State goal ) {
    m_Goal = goal;
}

void AngularMotorTuner::SetMotionProfile( tuning::AngularMotionProfile prof ) {
    delete m_Profile;
    m_Profile = new frc::TrapezoidProfile<units::degrees>({prof.MaxVelocity, prof.MaxAcceleration});
}


void AngularMotorTuner::CreateSoftwareFeedForward() {
    // Destroy old feedforwards.
    delete m_armFF;
    delete m_simpleFF;

    m_armFF = nullptr;
    m_simpleFF = nullptr;

        // Setup the Software feedforward.
    if( m_mech == tuning::Arm ) {
        // Using Arm
        m_armFF = new frc::ArmFeedforward{units::volt_t{ m_parameters.kS}, units::volt_t{ m_parameters.kG}, 
                                    units::unit_t<frc::ArmFeedforward::kv_unit> {m_parameters.kV}, 
                                    units::unit_t<frc::ArmFeedforward::ka_unit> {m_parameters.kA}};
    } else {
        // Use Elevator with zero kG for Simple
        m_simpleFF = new frc::SimpleMotorFeedforward<units::turn>( m_parameters.kS * 1_V, m_parameters.kV * 1_V * 1_s / 1_tr, 
                                                                m_parameters.kA * 1_V * 1_s * 1_s / 1_tr );
    }
}