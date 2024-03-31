
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Tuning.h"

using namespace tuning;

ParameterUI::ParameterUI( std::string tab, std::string name, tuning::Parameters p )
    : m_tab{tab}, m_name{name}, m_params{p} {

}

void ParameterUI::PutNTValues( ) {
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
}

tuning::Parameters ParameterUI::GetNTValues( ) {
  tuning::Parameters v;
  std::string item_name;

  item_name = std::string{m_name} + " P";
  v.kP = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " I";
  v.kI = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " D";
  v.kD = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " S";
  v.kS = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " G";
  v.kG = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " V";
  v.kV = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  item_name = std::string{m_name} + " A";
  v.kA = frc::SmartDashboard::GetNumber( item_name, 0.0 );

  return v;
}

MotorTuner::MotorTuner( std::string_view name, tuning::Parameters p, 
                        tuning::MechanismType mech, tuning::ControlType ctrl ) 
    : m_name{name}, m_parameters{p}, m_mech{mech}, m_ctrl{ctrl}, m_softPID{p.kP, p.kI, p.kD} {

    m_armFF = nullptr;
    m_elevatorFF = nullptr;
    m_simpleFF = nullptr;
    m_Profile = nullptr;

    if( m_ctrl == tuning::Software ) {
            // Setup the Software feedforward.
        CreateSoftwareFeedForward();

            // Default to some slow values.
        m_Profile = new frc::TrapezoidProfile<units::degrees>({60_deg_per_s, 60_deg_per_s_sq});
    }
}

void MotorTuner::Update( units::degree_t position, double arbFF ) {
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

void MotorTuner::Update( double arbFF ) {
    Update( GetPosition(), arbFF );
}

units::degree_t MotorTuner::GetPosition() {
    fmt::print( "MotorTuner::GetPosition() for <{}> NOT OVERRIDDEN!  CONTROL WILL NOT WORK!!!!\n", m_name );
    return 0_deg;
}

void MotorTuner::SetGoal( frc::TrapezoidProfile<units::degrees>::State goal ) {
    m_Goal = goal;
}

void MotorTuner::SetMotionProfile( tuning::MotionProfile prof ) {
    delete m_Profile;
    m_Profile = new frc::TrapezoidProfile<units::degrees>({prof.MaxVelocity, prof.MaxAcceleration});
}


void MotorTuner::CreateSoftwareFeedForward() {
    // Destroy old feedforwards.
    delete m_armFF;
    delete m_elevatorFF;
    delete m_simpleFF;

    m_armFF = nullptr;
    m_elevatorFF = nullptr;
    m_simpleFF = nullptr;

        // Setup the Software feedforward.
    if( m_mech == tuning::Arm ) {
        // Using Arm
        m_armFF = new frc::ArmFeedforward{units::volt_t{ m_parameters.kS}, units::volt_t{ m_parameters.kG}, 
                                    units::unit_t<frc::ArmFeedforward::kv_unit> {m_parameters.kV}, 
                                    units::unit_t<frc::ArmFeedforward::ka_unit> {m_parameters.kA}};
    } else if( m_mech == tuning::Elevator ) {
        // Using Elevator
        m_elevatorFF = new frc::ElevatorFeedforward{units::volt_t{ m_parameters.kS}, units::volt_t{ m_parameters.kG}, 
                                    units::unit_t<frc::ElevatorFeedforward::kv_unit> {m_parameters.kV}, 
                                    units::unit_t<frc::ElevatorFeedforward::ka_unit> {m_parameters.kA}};
    } else {
        // Use Elevator with zero kG for Simple
        m_simpleFF = new frc::SimpleMotorFeedforward<units::turn>( m_parameters.kS * 1_V, m_parameters.kV * 1_V * 1_s / 1_tr, 
                                                                m_parameters.kA * 1_V * 1_s * 1_s / 1_tr );
    }
}
