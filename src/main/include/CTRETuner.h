#pragma once

#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Tuning.h"

class BaseTalonFXTuner : virtual public tuning::BaseMotorTuner {
public:
    BaseTalonFXTuner( std::string_view name, int CAN_Id, std::string canbus, tuning::Parameters p, 
                      tuning::MechanismType mech, tuning::ControlType ctrl );

    void LinkCANCoder( int CANCoderID, double RotorToSensorRatio );
    void AddFollower( ctre::phoenix6::hardware::TalonFX &follower, bool invert=false );
    void SetParameters( tuning::Parameters p );
    void SetInverted( bool inverted = false );

protected:
    ctre::phoenix6::hardware::TalonFX m_talon;
    ctre::phoenix6::controls::MotionMagicDutyCycle m_request{0_deg};

    ctre::phoenix6::StatusSignal<units::turn_t> m_position = m_talon.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_velocity = m_talon.GetVelocity();
    ctre::phoenix6::StatusSignal<double> m_targetPos = m_talon.GetClosedLoopReference();
    ctre::phoenix6::StatusSignal<double> m_targetVel = m_talon.GetClosedLoopReferenceSlope();
};

class AngularTalonFXTuner : virtual public BaseTalonFXTuner, virtual public tuning::AngularMotorTuner {
public:
    AngularTalonFXTuner( std::string_view name, int CAN_Id, std::string canbus = "", tuning::Parameters p={}, 
                  tuning::MechanismType mech = tuning::Simple, tuning::ControlType ctrl = tuning::OnBoard );
    
    void SetMotionProfile( tuning::AngularMotionProfile prof );

    virtual units::degree_t GetPosition();
    void MotorPeriodic( double arbFF );
};
