
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "Tuning.h"

using namespace tuning;

AngularTuningUI::AngularTuningUI( std::string tab, std::string name, tuning::Parameters p, tuning::AngularMotionProfile prof )
    : m_tab{tab}, m_name{name}, m_params{p}, m_prof{prof} {

}

void AngularTuningUI::Refresh( ) {
    std::string item_name;

    frc::ShuffleboardLayout &layout = frc::Shuffleboard::GetTab(m_tab)
        .GetLayout(m_name, frc::BuiltInLayouts::kGrid)
        .WithProperties({ { "Number of columns", nt::Value::MakeDouble(2) },
                          { "Number of rows", nt::Value::MakeDouble(7) } })
        .WithSize(5, 7);

    item_name = std::string{m_name} + " P";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kP).WithPosition(0, 0).GetEntry();

    item_name = std::string{m_name} + " I";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kI).WithPosition(0, 1).GetEntry();

    item_name = std::string{m_name} + " D";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kD).WithPosition(0, 2).GetEntry();

    item_name = std::string{m_name} + " S";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kS).WithPosition(0, 3).GetEntry();

    item_name = std::string{m_name} + " G";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kG).WithPosition(0, 4).GetEntry();

    item_name = std::string{m_name} + " V";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kV).WithPosition(0, 5).GetEntry();

    item_name = std::string{m_name} + " A";
    m_nt_entries[item_name] = layout.Add(item_name, m_params.kA).WithPosition(0, 6).GetEntry();

    item_name = std::string{m_name} + " MaxVelocity";
    m_nt_entries[item_name] = layout.Add(item_name, m_prof.MaxVelocity.value()).WithPosition(1, 0).GetEntry();

    item_name = std::string{m_name} + " MaxAccel";
    m_nt_entries[item_name] = layout.Add(item_name, m_prof.MaxAcceleration.value()).WithPosition(1, 1).GetEntry();

    item_name = std::string{m_name} + " MaxJerk";
    m_nt_entries[item_name] = layout.Add(item_name, m_prof.MaxJerk.value()).WithPosition(1, 2).GetEntry();
}

tuning::Parameters AngularTuningUI::GetParameters( ) {
  std::string item_name;

  item_name = std::string{m_name} + " P";
  m_params.kP = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " I";
  m_params.kI = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " D";
  m_params.kD = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " S";
  m_params.kS = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " G";
  m_params.kG = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " V";
  m_params.kV = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " A";
  m_params.kA = m_nt_entries[item_name]->GetDouble( 0.0 );

  return m_params;
}

tuning::AngularMotionProfile AngularTuningUI::GetMotionProfile( ) {
  std::string item_name;

  item_name = std::string{m_name} + " MaxVelocity";
  m_prof.MaxVelocity = m_nt_entries[item_name]->GetDouble( 0.0 ) * 1_deg_per_s;

  item_name = std::string{m_name} + " MaxAccel";
  m_prof.MaxAcceleration = m_nt_entries[item_name]->GetDouble( 0.0 ) * 1_deg_per_s_sq;

  item_name = std::string{m_name} + " MaxJerk";
  m_prof.MaxJerk = m_nt_entries[item_name]->GetDouble( 0.0 ) * 1_deg_per_s_cu;

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
