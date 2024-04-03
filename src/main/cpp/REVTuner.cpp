
#include "REVTuner.h"

#include <units/voltage.h>

#include <frc/smartdashboard/SmartDashboard.h>

AngularSparkFlexTuner::AngularSparkFlexTuner( std::string_view name, int CAN_Id, std::string canbus, tuning::Parameters p, 
                            tuning::MechanismType mech, tuning::ControlType ctrl) 
    : AngularMotorTuner(name, p, mech, ctrl), m_flex{CAN_Id, rev::CANSparkLowLevel::MotorType::kBrushless}
{

    if( m_ctrl != tuning::Software ) {
            // Reset the configs.
        m_flex.RestoreFactoryDefaults();

        m_pidController.SetP(m_parameters.kP);
        m_pidController.SetI(m_parameters.kI);
        m_pidController.SetD(m_parameters.kD);
        m_pidController.SetIZone(0);
        m_pidController.SetFF(m_parameters.kV);
        m_pidController.SetOutputRange(-1, 1);    

        m_pidController.SetSmartMotionMaxVelocity( 30 );    /* in RPM */
        m_pidController.SetSmartMotionMaxAccel( 60 );    /* in RPM / second */
    }
}

void AngularSparkFlexTuner::SetParameters( tuning::Parameters p ) {
    m_parameters = p;
    if( m_ctrl == tuning::Software ) {
            // Setup the Software feedforward.
        CreateSoftwareFeedForward();        
    } else {

        m_pidController.SetP(m_parameters.kP);
        m_pidController.SetI(m_parameters.kI);
        m_pidController.SetD(m_parameters.kD);
        m_pidController.SetFF(m_parameters.kV);
    }
}

void AngularSparkFlexTuner::SetInverted( bool inverted ) {
    m_flex.SetInverted( inverted );
}

void AngularSparkFlexTuner::SetMotionProfile( tuning::AngularMotionProfile prof ) {
    if( m_ctrl == tuning::Software ) {
        AngularMotorTuner::SetMotionProfile( prof );
        return;
    }

    m_pidController.SetSmartMotionMaxVelocity(  units::revolutions_per_minute_t( prof.MaxVelocity ).value() );    /* in RPM */
    m_pidController.SetSmartMotionMaxAccel( units::revolutions_per_minute_squared_t( prof.MaxAcceleration ).value() / 60 );    /* in RPM / second */
}

void AngularSparkFlexTuner::MotorPeriodic( double arbFF ) {
    // Refresh all the signals.
    frc::SmartDashboard::PutNumber( m_name + " Position", m_encoder.GetPosition() * 360.0);
    frc::SmartDashboard::PutNumber( m_name + " Velocity(RPM)", m_encoder.GetVelocity());

    if( m_ctrl == tuning::OnBoard ) {
        frc::SmartDashboard::PutNumber( m_name + " SmartMotion Pos", m_Goal.position.value() );
        // frc::SmartDashboard::PutNumber( m_name + " SmartMotion Vel(RPM)", /* NO WAY TO GET THIS INFO */ );

        m_pidController.SetReference( units::turn_t(m_Goal.position).value(), rev::CANSparkFlex::ControlType::kSmartMotion );  

    } else {
        // Software control arbFF has the PID and FF calculation. 
        m_flex.Set( arbFF );
    }
}

units::degree_t AngularSparkFlexTuner::GetPosition() {
    return m_encoder.GetPosition() * 360_deg;
}