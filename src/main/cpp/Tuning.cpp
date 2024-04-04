
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "Tuning.h"

using namespace tuning;

AngularTuningUI::AngularTuningUI( std::string tab, std::string name, tuning::Parameters p, tuning::AngularMotionProfile prof )
    : m_tab{tab}, m_name{name} {

    std::string item_name;
    frc::ShuffleboardLayout &layout = frc::Shuffleboard::GetTab(m_tab)
        .GetLayout(m_name, frc::BuiltInLayouts::kGrid)
        .WithProperties({ { "Number of columns", nt::Value::MakeDouble(2) },
                          { "Number of rows", nt::Value::MakeDouble(7) } })
        .WithSize(5, 7);

    item_name = std::string{m_name} + " P";
    m_nt_entries[item_name] = layout.Add(item_name, p.kP).WithPosition(0, 0).GetEntry();

    item_name = std::string{m_name} + " I";
    m_nt_entries[item_name] = layout.Add(item_name, p.kI).WithPosition(0, 1).GetEntry();

    item_name = std::string{m_name} + " D";
    m_nt_entries[item_name] = layout.Add(item_name, p.kD).WithPosition(0, 2).GetEntry();

    item_name = std::string{m_name} + " S";
    m_nt_entries[item_name] = layout.Add(item_name, p.kS).WithPosition(0, 3).GetEntry();

    item_name = std::string{m_name} + " G";
    m_nt_entries[item_name] = layout.Add(item_name, p.kG).WithPosition(0, 4).GetEntry();

    item_name = std::string{m_name} + " V";
    m_nt_entries[item_name] = layout.Add(item_name, p.kV).WithPosition(0, 5).GetEntry();

    item_name = std::string{m_name} + " A";
    m_nt_entries[item_name] = layout.Add(item_name, p.kA).WithPosition(0, 6).GetEntry();

    item_name = std::string{m_name} + " MaxVelocity";
    m_nt_entries[item_name] = layout.Add(item_name, prof.MaxVelocity.value()).WithPosition(1, 0).GetEntry();

    item_name = std::string{m_name} + " MaxAccel";
    m_nt_entries[item_name] = layout.Add(item_name, prof.MaxAcceleration.value()).WithPosition(1, 1).GetEntry();

    item_name = std::string{m_name} + " MaxJerk";
    m_nt_entries[item_name] = layout.Add(item_name, prof.MaxJerk.value()).WithPosition(1, 2).GetEntry();
}

void AngularTuningUI::SetParameters( tuning::Parameters p ) {
    std::string item_name;

    item_name = std::string{m_name} + " P";
    m_nt_entries[item_name]->SetDouble( p.kP );

    item_name = std::string{m_name} + " I";
    m_nt_entries[item_name]->SetDouble( p.kI );

    item_name = std::string{m_name} + " D";
    m_nt_entries[item_name]->SetDouble( p.kD );

    item_name = std::string{m_name} + " S";
    m_nt_entries[item_name]->SetDouble( p.kS );

    item_name = std::string{m_name} + " G";
    m_nt_entries[item_name]->SetDouble( p.kG );

    item_name = std::string{m_name} + " V";
    m_nt_entries[item_name]->SetDouble( p.kV );

    item_name = std::string{m_name} + " A";
    m_nt_entries[item_name]->SetDouble( p.kA );
}

tuning::Parameters AngularTuningUI::GetParameters( ) {
  std::string item_name;
  tuning::Parameters p;

  item_name = std::string{m_name} + " P";
  p.kP = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " I";
  p.kI = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " D";
  p.kD = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " S";
  p.kS = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " G";
  p.kG = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " V";
  p.kV = m_nt_entries[item_name]->GetDouble( 0.0 );

  item_name = std::string{m_name} + " A";
  p.kA = m_nt_entries[item_name]->GetDouble( 0.0 );

  return p;
}

tuning::AngularMotionProfile AngularTuningUI::GetMotionProfile( ) {
  std::string item_name;
  tuning::AngularMotionProfile prof;

  item_name = std::string{m_name} + " MaxVelocity";
  prof.MaxVelocity = m_nt_entries[item_name]->GetDouble( 0.0 ) * 1_deg_per_s;

  item_name = std::string{m_name} + " MaxAccel";
  prof.MaxAcceleration = m_nt_entries[item_name]->GetDouble( 0.0 ) * 1_deg_per_s_sq;

  item_name = std::string{m_name} + " MaxJerk";
  prof.MaxJerk = m_nt_entries[item_name]->GetDouble( 0.0 ) * 1_deg_per_s_cu;

  return prof;
}

void AngularTuningUI::SetMotionProfile( tuning::AngularMotionProfile prof ) { 
    std::string item_name;

    item_name = std::string{m_name} + " MaxVelocity";
    m_nt_entries[item_name]->SetDouble( prof.MaxVelocity.value() );

    item_name = std::string{m_name} + " MaxAccel";
    m_nt_entries[item_name]->SetDouble( prof.MaxAcceleration.value() );

    item_name = std::string{m_name} + " MaxJerk";
    m_nt_entries[item_name]->SetDouble( prof.MaxJerk.value() );
}




BaseMotorTuner::BaseMotorTuner( 
    std::string_view name, tuning::Parameters p, tuning::MechanismType mech, tuning::ControlType ctrl ) :
    m_name{name}, m_parameters{p}, m_mech{mech}, m_ctrl{ctrl}, m_softPID{p.kP, p.kI, p.kD} {

}

AngularMotorTuner::AngularMotorTuner( std::string_view name, tuning::Parameters p, 
                        tuning::MechanismType mech, tuning::ControlType ctrl ) 
    : BaseMotorTuner( name, p, mech, ctrl ) {

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

        double motorOutput, ffOutput;
        motorOutput = m_softPID.Calculate( position.value(), m_Setpoint.position.value() );
        if( m_mech == tuning::Arm ) {
            ffOutput = m_armFF->Calculate( m_Setpoint.position, m_Setpoint.velocity).value();
        } else {
            ffOutput = m_simpleFF->Calculate( m_Setpoint.velocity ).value();
        }
        frc::SmartDashboard::PutNumber( m_name + " PID Out", motorOutput);
        frc::SmartDashboard::PutNumber( m_name + " Feedforward Out", ffOutput );
        frc::SmartDashboard::PutNumber( m_name + " Setpoint Position", m_Setpoint.position.value());
        frc::SmartDashboard::PutNumber( m_name + " Setpoint Velocity(RPM)", units::revolutions_per_minute_t(m_Setpoint.velocity).value() );

        arbFF += motorOutput + ffOutput / 12.0;
    }

    MotorPeriodic( arbFF );
}

void AngularMotorTuner::Update( double arbFF ) {
    Update( GetPosition(), arbFF );
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
        // Using Arm FF
        m_armFF = new frc::ArmFeedforward{units::volt_t{ m_parameters.kS}, units::volt_t{ m_parameters.kG}, 
                                    units::unit_t<frc::ArmFeedforward::kv_unit> {m_parameters.kV}, 
                                    units::unit_t<frc::ArmFeedforward::ka_unit> {m_parameters.kA}};
    } else {
        // Use Simple FF
        m_simpleFF = new frc::SimpleMotorFeedforward<units::degrees>( m_parameters.kS * 1_V, m_parameters.kV * 1_V / 1_deg_per_s, 
                                                                      m_parameters.kA * 1_V / 1_deg_per_s_sq );
    }
}


LinearMotorTuner::LinearMotorTuner( std::string_view name, tuning::Parameters p, MetersPerTurn gearRatio,
                        tuning::MechanismType mech, tuning::ControlType ctrl ) 
    : BaseMotorTuner( name, p, mech, ctrl ), m_gearRatio{gearRatio} {

    m_elevatorFF = nullptr;
    m_simpleFF = nullptr;
    m_Profile = nullptr;

    if( m_ctrl == tuning::Software ) {
            // Setup the Software feedforward.
        CreateSoftwareFeedForward();

            // Default to some slow values.
        m_Profile = new frc::TrapezoidProfile<units::meters>({1_mps, 1_mps_sq});
    }
}

void LinearMotorTuner::Update( units::meter_t position, double arbFF ) {
    frc::SmartDashboard::PutNumber( m_name + " Position", position.value());
    frc::SmartDashboard::PutNumber( m_name + " Goal Position", m_Goal.position.value());
    
    if ( frc::DriverStation::IsDisabled() ) {
        m_Setpoint.position = position;
        m_Setpoint.velocity = 0_mps;
        m_Goal = { m_Setpoint.position, 0_mps}; 
        return;
    }

    if( m_ctrl == tuning::Software ) {
        m_Setpoint = m_Profile->Calculate(20_ms, m_Setpoint, m_Goal);

        double motorOutput, ffOutput;
        motorOutput = m_softPID.Calculate( position.value(), m_Setpoint.position.value() );
        if( m_mech == tuning::Elevator ) {
            ffOutput = m_elevatorFF->Calculate( m_Setpoint.velocity ).value();
        } else {
            ffOutput = m_simpleFF->Calculate( m_Setpoint.velocity ).value();
        }
        frc::SmartDashboard::PutNumber( m_name + " PID Out", motorOutput);
        frc::SmartDashboard::PutNumber( m_name + " Feedforward Out", ffOutput );
        frc::SmartDashboard::PutNumber( m_name + " Setpoint Position", m_Setpoint.position.value());
        frc::SmartDashboard::PutNumber( m_name + " Setpoint Velocity(mps)", m_Setpoint.velocity.value());

        arbFF += motorOutput + ffOutput / 12.0;
    }

    MotorPeriodic( arbFF );
}

void LinearMotorTuner::Update( double arbFF ) {
    Update( GetPosition(), arbFF );
}

void LinearMotorTuner::SetGoal( frc::TrapezoidProfile<units::meters>::State goal ) {
    m_Goal = goal;
}

void LinearMotorTuner::SetGearRatio( MetersPerTurn gearRatio ) {
    m_gearRatio = gearRatio;
}

void LinearMotorTuner::SetMotionProfile( tuning::LinearMotionProfile prof ) {
    delete m_Profile;
    m_Profile = new frc::TrapezoidProfile<units::meters>({prof.MaxVelocity, prof.MaxAcceleration});
}


void LinearMotorTuner::CreateSoftwareFeedForward() {
    // Destroy old feedforwards.
    delete m_elevatorFF;
    delete m_simpleFF;

    m_elevatorFF = nullptr;
    m_simpleFF = nullptr;

        // Setup the Software feedforward.
    if( m_mech == tuning::Elevator ) {
        // Using Arm
        m_elevatorFF = new frc::ElevatorFeedforward{units::volt_t{ m_parameters.kS}, units::volt_t{ m_parameters.kG}, 
                                    units::unit_t<frc::ElevatorFeedforward::kv_unit> {m_parameters.kV}, 
                                    units::unit_t<frc::ElevatorFeedforward::ka_unit> {m_parameters.kA}};
    } else {
        // Use Elevator with zero kG for Simple
        m_simpleFF = new frc::SimpleMotorFeedforward<units::meters>( m_parameters.kS * 1_V, m_parameters.kV * 1_V / 1_mps, 
                                                                     m_parameters.kA * 1_V / 1_mps_sq );
    }
}


VelocityMotorTuner::VelocityMotorTuner( std::string_view name, tuning::Parameters p, tuning::ControlType ctrl ) 
    : BaseMotorTuner( name, p, tuning::Simple, ctrl ) {

    m_simpleFF = nullptr;

    if( m_ctrl == tuning::Software ) {
            // Setup the Software feedforward.
        CreateSoftwareFeedForward();
    }
}

void VelocityMotorTuner::Update( units::revolutions_per_minute_t velocity, double arbFF ) {
    frc::SmartDashboard::PutNumber( m_name + " Velocity", velocity.value());
    frc::SmartDashboard::PutNumber( m_name + " Goal Velocity", m_Goal.value());
    
    if ( frc::DriverStation::IsDisabled() ) {
        m_Goal = 0_rpm; 
        return;
    }

    if( m_ctrl == tuning::Software ) {

        double motorOutput, ffOutput;
        motorOutput = m_softPID.Calculate( velocity.value(), m_Goal.value() );
        ffOutput = m_simpleFF->Calculate( velocity ).value();
        frc::SmartDashboard::PutNumber( m_name + " PID Out", motorOutput);
        frc::SmartDashboard::PutNumber( m_name + " Feedforward Out", ffOutput );

        arbFF += motorOutput + ffOutput / 12.0;
    }

    MotorPeriodic( arbFF );
}

void VelocityMotorTuner::Update( double arbFF ) {
    Update( GetVelocity(), arbFF );
}

void VelocityMotorTuner::SetVelocity( units::revolutions_per_minute_t goal ) {
    m_Goal = goal;
}

void VelocityMotorTuner::CreateSoftwareFeedForward() {
    // Destroy old feedforwards.
    delete m_simpleFF;

    m_simpleFF = nullptr;

        // Setup the Software feedforward.
    m_simpleFF = new frc::SimpleMotorFeedforward<units::turns>( m_parameters.kS * 1_V, m_parameters.kV * 1_V * 1_s / 1_tr, 
                                                                m_parameters.kA * 1_V * 1_s * 1_s / 1_tr );
}
