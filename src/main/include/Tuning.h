#pragma once

#include <string>
#include <units/angular_jerk.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

namespace tuning { 

struct Parameters {
    double kP;
    double kI;
    double kD;
    double kS;
    double kG;
    double kV;
    double kA;
};

class MotionProfile {
public:
    units::degrees_per_second_t MaxVelocity;
    units::degrees_per_second_squared_t MaxAcceleration;
    units::degrees_per_second_cubed_t MaxJerk;
};

class ParameterUI {
public:
    ParameterUI( std::string tab, std::string name, tuning::Parameters p );

    void PutNTValues( );
    tuning::Parameters GetNTValues( );
private:
    std::string m_tab;
    std::string m_name;
    tuning::Parameters m_params;
};

enum MechanismType { Simple, Elevator, Arm };
enum ControlType { OnBoard, Software };

class MotorTuner {
public:
    MotorTuner( std::string_view name, tuning::Parameters p, 
                tuning::MechanismType mech, tuning::ControlType ctrl );

    void Update( units::degree_t position, double arbFF = 0.0 );
    void Update( double arbFF = 0.0 );

    virtual void SetGoal( frc::TrapezoidProfile<units::degrees>::State goal );

    virtual void SetParameters( tuning::Parameters p ) = 0;
    virtual tuning::Parameters GetParameters( void ) {return m_parameters;}

    virtual void SetInverted( bool inverted = false ) = 0;

    virtual void SetMotionProfile( tuning::MotionProfile prof );

protected:
    std::string m_name;
    tuning::Parameters m_parameters;
    tuning::MechanismType m_mech;
    tuning::ControlType m_ctrl;

        // Software Control Stuff
    frc::PIDController m_softPID;
    frc::SimpleMotorFeedforward<units::turn> *m_simpleFF;
    frc::ElevatorFeedforward *m_elevatorFF;
    frc::ArmFeedforward *m_armFF;

        // Software Trapezoid Profile
    frc::TrapezoidProfile<units::degrees> *m_Profile;
    frc::TrapezoidProfile<units::degrees>::State m_Goal;
    frc::TrapezoidProfile<units::degrees>::State m_Setpoint;

    void CreateSoftwareFeedForward();

        // Must override to provide current position value that control is based on.
    virtual units::degree_t GetPosition();
    virtual void MotorPeriodic( double arbFF ) = 0;
};

}  // namespace tuning