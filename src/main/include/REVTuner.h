#pragma once

#include <string>

#include <rev/CANSparkFlex.h>

#include "Tuning.h"

class AngularSparkFlexTuner : public tuning::AngularMotorTuner {
public:
    AngularSparkFlexTuner( std::string_view name, int CAN_Id, std::string canbus = "", tuning::Parameters p={}, 
                  tuning::MechanismType mech = tuning::Simple, tuning::ControlType ctrl = tuning::OnBoard );
    
    void SetParameters( tuning::Parameters p );
    void SetInverted( bool inverted = false );
    void SetMotionProfile( tuning::AngularMotionProfile prof );

private:
    rev::CANSparkFlex m_flex;
    rev::SparkRelativeEncoder m_encoder = m_flex.GetEncoder();
    rev::SparkPIDController m_pidController = m_flex.GetPIDController();

    virtual units::degree_t GetPosition();
    void MotorPeriodic( double arbFF );
};
