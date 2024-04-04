#pragma once

#include <string>

#include <rev/CANSparkFlex.h>

#include "Tuning.h"

class BaseSparkFlexTuner : virtual public tuning::BaseMotorTuner {
public:
    BaseSparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p, 
                        tuning::MechanismType mech, tuning::ControlType ctrl );

    void SetParameters( tuning::Parameters p );
    void SetInverted( bool inverted = false );
protected:
    rev::CANSparkFlex m_flex;
    rev::SparkRelativeEncoder m_encoder = m_flex.GetEncoder();
    rev::SparkPIDController m_pidController = m_flex.GetPIDController();
};

class AngularSparkFlexTuner : virtual public BaseSparkFlexTuner, virtual public tuning::AngularMotorTuner {
public:
    AngularSparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p={}, 
                  tuning::MechanismType mech = tuning::Simple, tuning::ControlType ctrl = tuning::OnBoard );
    
    // void SetParameters( tuning::Parameters p );
    // void SetInverted( bool inverted = false );
    void SetMotionProfile( tuning::AngularMotionProfile prof );

private:
    // rev::CANSparkFlex m_flex;
    // rev::SparkRelativeEncoder m_encoder = m_flex.GetEncoder();
    // rev::SparkPIDController m_pidController = m_flex.GetPIDController();

    virtual units::degree_t GetPosition();
    void MotorPeriodic( double arbFF );
};

class LinearSparkFlexTuner : public tuning::LinearMotorTuner {
public:
    LinearSparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p={}, MetersPerTurn gearRatio = 1_m / 1_tr,
                  tuning::MechanismType mech = tuning::Simple, tuning::ControlType ctrl = tuning::OnBoard );
    
    // void SetParameters( tuning::Parameters p );
    // void SetInverted( bool inverted = false );
    void SetMotionProfile( tuning::LinearMotionProfile prof );

private:
    // rev::CANSparkFlex m_flex;
    // rev::SparkRelativeEncoder m_encoder = m_flex.GetEncoder();
    // rev::SparkPIDController m_pidController = m_flex.GetPIDController();

    virtual units::meter_t GetPosition();
    void MotorPeriodic( double arbFF );
};

class VelocitySparkFlexTuner : virtual public BaseSparkFlexTuner, virtual public tuning::VelocityMotorTuner {
public:
    VelocitySparkFlexTuner( std::string_view name, int CAN_Id, tuning::Parameters p={}, 
                            tuning::ControlType ctrl = tuning::OnBoard );

private:

    virtual units::revolutions_per_minute_t GetVelocity();
    void MotorPeriodic( double arbFF );
};
