
#include "CTRETuner.h"

#include <units/voltage.h>

#include <frc/smartdashboard/SmartDashboard.h>

BaseTalonFXTuner::BaseTalonFXTuner( std::string_view name, int CAN_Id, std::string canbus, tuning::Parameters p, 
                                    tuning::MechanismType mech, tuning::ControlType ctrl ) 
    : BaseMotorTuner( name, p, mech, ctrl ), m_talon{CAN_Id, canbus}
{
    m_position.SetUpdateFrequency(50_Hz);
    m_velocity.SetUpdateFrequency(50_Hz);
    m_targetPos.SetUpdateFrequency(50_Hz);
    m_targetVel.SetUpdateFrequency(50_Hz);

    if( m_ctrl != tuning::Software ) {
            // Reset the configs.
        m_talon.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

        ctre::phoenix6::configs::TalonFXConfiguration configs{};
        switch( m_mech ) {
            case tuning::Arm:
                configs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
                break;
            default:
                configs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
                break;
        }
        configs.Slot0.kP = m_parameters.kP;
        configs.Slot0.kI = m_parameters.kI;
        configs.Slot0.kD = m_parameters.kD;
        configs.Slot0.kS = m_parameters.kS;
        configs.Slot0.kG = m_parameters.kG;
        configs.Slot0.kV = m_parameters.kV;
        configs.Slot0.kA = m_parameters.kA;
        m_talon.GetConfigurator().Apply(configs);
    }
}

void BaseTalonFXTuner::LinkCANCoder( int CANCoderID, double RotorToSensorRatio ) {

    ctre::phoenix6::configs::TalonFXConfiguration configs{};
    m_talon.GetConfigurator().Refresh(configs);

    configs.Feedback.FeedbackRemoteSensorID = CANCoderID;
    configs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    configs.Feedback.RotorToSensorRatio = RotorToSensorRatio;
    m_talon.GetConfigurator().Apply(configs);
    fmt::print( "BaseTalonFXTuner::LinkCANCoder -- <{}> was linked with CANCoder ID = {}.\n", m_name, CANCoderID );
}

void BaseTalonFXTuner::AddFollower( ctre::phoenix6::hardware::TalonFX &follower, bool invert ) {
    follower.SetControl( ctre::phoenix6::controls::Follower{ m_talon.GetDeviceID(), invert } );
}

void BaseTalonFXTuner::SetParameters( tuning::Parameters p ) {
    m_parameters = p;
    if( m_ctrl == tuning::Software ) {
            // Setup the Software feedforward.
        CreateSoftwareFeedForward();
        
    } else {
        ctre::phoenix6::configs::TalonFXConfiguration configs{};
        m_talon.GetConfigurator().Refresh(configs);

        configs.Slot0.kP = m_parameters.kP;
        configs.Slot0.kI = m_parameters.kI;
        configs.Slot0.kD = m_parameters.kD;
        configs.Slot0.kS = m_parameters.kS;
        configs.Slot0.kG = m_parameters.kG;
        configs.Slot0.kV = m_parameters.kV;
        configs.Slot0.kA = m_parameters.kA;
        m_talon.GetConfigurator().Apply(configs);
        fmt::print( "AngularTalonFXTuner::SetParameters -- <{}> Talon PID+SGVA configured.\n", m_name );
    }
}

void BaseTalonFXTuner::SetInverted( bool inverted ) {
    ctre::phoenix6::configs::TalonFXConfiguration configs{};
    m_talon.GetConfigurator().Refresh(configs);
    configs.MotorOutput.Inverted = inverted;
    m_talon.GetConfigurator().Apply(configs);
}




AngularTalonFXTuner::AngularTalonFXTuner( std::string_view name, int CAN_Id, std::string canbus, tuning::Parameters p, 
                            tuning::MechanismType mech, tuning::ControlType ctrl) 
    : BaseMotorTuner( name, p, mech, ctrl ),
      BaseTalonFXTuner( name, CAN_Id, canbus, p, mech, ctrl ),
      AngularMotorTuner(name, p, mech, ctrl)
{
    if( m_ctrl != tuning::Software ) {
            // Set the Motion Magic values for Angular motion.
        ctre::phoenix6::configs::TalonFXConfiguration configs{};
        m_talon.GetConfigurator().Refresh(configs);
        configs.MotionMagic.MotionMagicCruiseVelocity = 0.5;    /* in rotations per second */
        configs.MotionMagic.MotionMagicAcceleration = 1;        /* in rotations per second^2 */
        configs.MotionMagic.MotionMagicJerk = 2;                /* in rotations per second^3 */
        m_talon.GetConfigurator().Apply(configs);
        fmt::print( "AngularTalonFXTuner::AngularTalonFXTuner -- <{}> Talon MotionMagic configured.\n", m_name );
    }
}

void AngularTalonFXTuner::SetMotionProfile( tuning::AngularMotionProfile prof ) {
    if( m_ctrl == tuning::Software ) {
        AngularMotorTuner::SetMotionProfile( prof );
        return;
    }

    ctre::phoenix6::configs::TalonFXConfiguration configs{};
    m_talon.GetConfigurator().Refresh(configs);

    configs.MotionMagic.MotionMagicCruiseVelocity = prof.MaxVelocity.value() / 360.0;       /* in rotations per second */
    configs.MotionMagic.MotionMagicAcceleration = prof.MaxAcceleration.value() / 360.0;     /* in rotations per second^2 */
    configs.MotionMagic.MotionMagicJerk = prof.MaxJerk.value() / 360.0;                     /* in rotations per second^3 */
    m_talon.GetConfigurator().Apply(configs);
    fmt::print( "AngularTalonFXTuner::SetMotionProfile -- <{}> Talon MotionMagic configured.\n", m_name );
}

void AngularTalonFXTuner::MotorPeriodic( double arbFF ) {
    // Refresh all the signals.
    m_position.Refresh();
    m_velocity.Refresh();
// output in Update()    frc::SmartDashboard::PutNumber( m_name + " Talon Position", m_position.GetValueAsDouble() * 360.0);
    frc::SmartDashboard::PutNumber( m_name + " Talon Velocity(RPM)", m_velocity.GetValueAsDouble() * 60.0);

    if( m_ctrl == tuning::OnBoard ) {
        m_targetPos.Refresh();
        m_targetVel.Refresh();
        frc::SmartDashboard::PutNumber( m_name + " MMagic Ref Pos", m_targetPos.GetValueAsDouble() * 360.0);
        frc::SmartDashboard::PutNumber( m_name + " MMagic Ref Vel(RPM)", m_targetVel.GetValueAsDouble() * 60.0);

        m_talon.SetControl( m_request.WithPosition( m_Goal.position ).WithFeedForward(arbFF) );  

    } else {
        // Software control arbFF has the PID and FF calculation. 
        m_talon.Set( arbFF );
    }
    frc::SmartDashboard::PutNumber( m_name + " Voltage", m_talon.GetMotorVoltage().GetValueAsDouble() );
    frc::SmartDashboard::PutNumber( m_name + " Current", m_talon.GetSupplyCurrent().GetValueAsDouble() );
}

units::degree_t AngularTalonFXTuner::GetPosition() {
    return m_position.GetValue();
}