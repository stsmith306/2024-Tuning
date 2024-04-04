
#include "REVTuner.h"

#include <units/voltage.h>

#include <frc/smartdashboard/SmartDashboard.h>

BaseSparkFlexTuner::BaseSparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p, 
                              tuning::MechanismType mech, tuning::ControlType ctrl ) :
    BaseMotorTuner( name, p, mech, ctrl ), m_flex{CAN_Id, rev::CANSparkLowLevel::MotorType::kBrushless} 
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
    }    
}

void BaseSparkFlexTuner::SetInverted( bool inverted ) {
    m_flex.SetInverted( inverted );
}

void BaseSparkFlexTuner::SetParameters( tuning::Parameters p ) {
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




AngularSparkFlexTuner::AngularSparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p, 
                            tuning::MechanismType mech, tuning::ControlType ctrl) 
    : BaseMotorTuner( name, p, mech, ctrl ), 
      BaseSparkFlexTuner( name, CAN_Id, p, mech, ctrl ), 
      AngularMotorTuner( name, p, mech, ctrl )
{

    if( m_ctrl != tuning::Software ) {
            // setup the Angular specific smart motion values.
        m_pidController.SetSmartMotionMaxVelocity( 30 );    /* in RPM */
        m_pidController.SetSmartMotionMaxAccel( 60 );    /* in RPM / second */
    }
}


void AngularSparkFlexTuner::SetMotionProfile( tuning::AngularMotionProfile prof ) {
    if( m_ctrl == tuning::Software ) {
        AngularMotorTuner::SetMotionProfile( prof );
        return;
    }

    m_pidController.SetSmartMotionMaxVelocity(  units::revolutions_per_minute_t( prof.MaxVelocity ).value() );    /* in RPM */
    m_pidController.SetSmartMotionMaxAccel( units::revolutions_per_minute_per_second_t(prof.MaxAcceleration).value() );    /* in RPM / second */
}

void AngularSparkFlexTuner::MotorPeriodic( double arbFF ) {
    // Refresh all the signals.
    frc::SmartDashboard::PutNumber( m_name + " EncPosition", m_encoder.GetPosition() * 360.0);
    frc::SmartDashboard::PutNumber( m_name + " Velocity(RPM)", m_encoder.GetVelocity());

    if( m_ctrl == tuning::OnBoard ) {
        frc::SmartDashboard::PutNumber( m_name + " SmartMotion Pos", m_Goal.position.value() );
        // frc::SmartDashboard::PutNumber( m_name + " SmartMotion Vel(RPM)", /* NO WAY TO GET THIS INFO */ );

        m_pidController.SetReference( units::turn_t(m_Goal.position).value(), rev::CANSparkFlex::ControlType::kSmartMotion );  

    } else {
        // Software control arbFF has the PID and FF calculation. 
        m_flex.Set( arbFF );
    }
    frc::SmartDashboard::PutNumber( m_name + " DutyCycle", m_flex.GetAppliedOutput() );
    frc::SmartDashboard::PutNumber( m_name + " Current", m_flex.GetOutputCurrent() );
}

units::degree_t AngularSparkFlexTuner::GetPosition() {
    return m_encoder.GetPosition() * 360_deg;
}



VelocitySparkFlexTuner::VelocitySparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p, 
                                                tuning::ControlType ctrl ) 
    : BaseMotorTuner( name, p, tuning::Simple, ctrl ),
      BaseSparkFlexTuner( name, CAN_Id, p, tuning::Simple, ctrl )
{

}

void VelocitySparkFlexTuner::MotorPeriodic( double arbFF ) {
    // Refresh all the signals.
    // frc::SmartDashboard::PutNumber( m_name + " Position", m_encoder.GetPosition() * 360.0);
    // frc::SmartDashboard::PutNumber( m_name + " Velocity(RPM)", m_encoder.GetVelocity());

    if( m_ctrl == tuning::OnBoard ) {
        m_pidController.SetReference( m_encoder.GetVelocity(), rev::CANSparkFlex::ControlType::kVelocity );  

    } else {
        // Software control arbFF has the PID and FF calculation. 
        m_flex.Set( arbFF );
    }
    frc::SmartDashboard::PutNumber( m_name + " DutyCycle", m_flex.GetAppliedOutput() );
    frc::SmartDashboard::PutNumber( m_name + " Current", m_flex.GetOutputCurrent() );
}

units::revolutions_per_minute_t VelocitySparkFlexTuner::GetVelocity() {
    return m_encoder.GetVelocity() * 1_rpm;
}

