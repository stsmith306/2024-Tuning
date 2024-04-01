#pragma once

#include <string>
#include <units/angular_jerk.h>
#include <units/acceleration.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <networktables/GenericEntry.h>

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

class AngularMotionProfile {
public:
    units::degrees_per_second_t MaxVelocity;
    units::degrees_per_second_squared_t MaxAcceleration;
    units::degrees_per_second_cubed_t MaxJerk;
};

class LinearMotionProfile {
public:
    using Jerk = units::compound_unit<units::meter, units::inverse<units::cubed<units::seconds>>>;

    units::meters_per_second_t MaxVelocity;
    units::meters_per_second_squared_t MaxAcceleration;
    Jerk MaxJerk;
};

class AngularTuningUI {
public:
    AngularTuningUI( std::string tab, std::string name, tuning::Parameters p, tuning::AngularMotionProfile prof );

    void Refresh( );
    tuning::Parameters GetParameters( );
    tuning::AngularMotionProfile GetMotionProfile( );

private:
    std::string m_tab;
    std::string m_name;
public:
    tuning::Parameters m_params;
    tuning::AngularMotionProfile m_prof;
    std::map<std::string, nt::GenericEntry*> m_nt_entries;
};



enum MechanismType { Simple, Elevator, Arm };
enum ControlType { OnBoard, Software };

class AngularMotorTuner {
public:
    AngularMotorTuner( std::string_view name, tuning::Parameters p, 
                tuning::MechanismType mech, tuning::ControlType ctrl );

        // Update and provide the position to use for control.
    void Update( units::degree_t position, double arbFF = 0.0 );

        // Update and use the default encoder for position control.
    void Update( double arbFF = 0.0 );

    virtual void SetGoal( frc::TrapezoidProfile<units::degrees>::State goal );

    virtual void SetParameters( tuning::Parameters p ) = 0;
    virtual tuning::Parameters GetParameters( void ) {return m_parameters;}

    virtual void SetInverted( bool inverted = false ) = 0;

    virtual void SetMotionProfile( tuning::AngularMotionProfile prof );

protected:
    std::string m_name;
    tuning::Parameters m_parameters;
    tuning::MechanismType m_mech;
    tuning::ControlType m_ctrl;

        // Software Control Stuff
    frc::PIDController m_softPID;
    frc::SimpleMotorFeedforward<units::turn> *m_simpleFF;
    frc::ArmFeedforward *m_armFF;

        // Software Angular Trapezoid Profile
    frc::TrapezoidProfile<units::degrees> *m_Profile;
    frc::TrapezoidProfile<units::degrees>::State m_Goal;
    frc::TrapezoidProfile<units::degrees>::State m_Setpoint;

    void CreateSoftwareFeedForward();

        // Must override to provide current position value that control is based on.
    virtual units::degree_t GetPosition();
    virtual void MotorPeriodic( double arbFF ) = 0;
};

class LinearMotorTuner {
public:
    LinearMotorTuner( std::string_view name, tuning::Parameters p, 
                tuning::MechanismType mech, tuning::ControlType ctrl );

        // Update and provide the position to use for control.
    void Update( units::meter_t position, double arbFF = 0.0 );

        // Update and use the default encoder for position control.
    void Update( double arbFF = 0.0 );

    virtual void SetGoal( frc::TrapezoidProfile<units::meters>::State goal );

    virtual void SetParameters( tuning::Parameters p ) = 0;
    virtual tuning::Parameters GetParameters( void ) {return m_parameters;}

    virtual void SetInverted( bool inverted = false ) = 0;

    virtual void SetMotionProfile( tuning::LinearMotionProfile prof );

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

        // Software Linear Trapezoid Profile
    frc::TrapezoidProfile<units::meters> *m_Profile;
    frc::TrapezoidProfile<units::meters>::State m_Goal;
    frc::TrapezoidProfile<units::meters>::State m_Setpoint;

    void CreateSoftwareFeedForward();

        // Must override to provide current position value that control is based on.
    virtual units::meter_t GetPosition();
    virtual void MotorPeriodic( double arbFF ) = 0;
};

}  // namespace tuning