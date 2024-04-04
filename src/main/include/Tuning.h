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

    tuning::Parameters GetParameters( );
    void SetParameters( tuning::Parameters );

    tuning::AngularMotionProfile GetMotionProfile( );
    void SetMotionProfile( tuning::AngularMotionProfile );

private:
    std::string m_tab;
    std::string m_name;
    std::map<std::string, nt::GenericEntry*> m_nt_entries;
};



enum MechanismType { Simple, Elevator, Arm };
enum ControlType { OnBoard, Software };

class BaseMotorTuner {
public:
    BaseMotorTuner( std::string_view name, tuning::Parameters p, 
                tuning::MechanismType mech, tuning::ControlType ctrl );

    virtual void SetParameters( tuning::Parameters p ) = 0;
    virtual tuning::Parameters GetParameters( void ) {return m_parameters;}

    virtual void SetInverted( bool inverted = false ) = 0;

    virtual void SetMotionProfile( tuning::AngularMotionProfile prof ) = 0;
protected:
    std::string m_name;
    tuning::Parameters m_parameters;
    tuning::MechanismType m_mech;
    tuning::ControlType m_ctrl;

        // Software Control Stuff
    frc::PIDController m_softPID;

    virtual void MotorPeriodic( double arbFF ) = 0;
    virtual void CreateSoftwareFeedForward() = 0;
};

class AngularMotorTuner : virtual public BaseMotorTuner {
public:
    AngularMotorTuner( std::string_view name, tuning::Parameters p, 
                tuning::MechanismType mech, tuning::ControlType ctrl );

        // Update and provide the position to use for control.
    void Update( units::degree_t position, double arbFF = 0.0 );

        // Update and use the default encoder for position control.
    void Update( double arbFF = 0.0 );

    virtual void SetGoal( frc::TrapezoidProfile<units::degrees>::State goal );

    virtual void SetMotionProfile( tuning::AngularMotionProfile prof );

protected:
         // Software Control Stuff
    frc::SimpleMotorFeedforward<units::degrees> *m_simpleFF;
    frc::ArmFeedforward *m_armFF;

        // Software Angular Trapezoid Profile
    frc::TrapezoidProfile<units::degrees> *m_Profile;
    frc::TrapezoidProfile<units::degrees>::State m_Goal;
    frc::TrapezoidProfile<units::degrees>::State m_Setpoint;

    void CreateSoftwareFeedForward();

        // Must override to provide current position value that control is based on.
    virtual units::degree_t GetPosition() = 0;
};

class LinearMotorTuner : virtual public BaseMotorTuner {
public:
    using MetersPerTurn = units::unit_t<units::compound_unit<units::meters, units::inverse<units::turns>>>;

    LinearMotorTuner( std::string_view name, tuning::Parameters p, MetersPerTurn gearRatio,
                tuning::MechanismType mech, tuning::ControlType ctrl );

        // Update and provide the position to use for control.
    void Update( units::meter_t position, double arbFF = 0.0 );

        // Update and use the default encoder for position control.
    void Update( double arbFF = 0.0 );

    virtual void SetGoal( frc::TrapezoidProfile<units::meters>::State goal );

    void SetGearRatio( MetersPerTurn gearRatio );

    virtual void SetMotionProfile( tuning::LinearMotionProfile prof );

protected:
        // Gear ratio between encoder rotations and linear motion
    MetersPerTurn m_gearRatio;

        // Software Control Stuff
    frc::SimpleMotorFeedforward<units::meters> *m_simpleFF;
    frc::ElevatorFeedforward *m_elevatorFF;

        // Software Linear Trapezoid Profile
    frc::TrapezoidProfile<units::meters> *m_Profile;
    frc::TrapezoidProfile<units::meters>::State m_Goal;
    frc::TrapezoidProfile<units::meters>::State m_Setpoint;

    void CreateSoftwareFeedForward();

        // Must override to provide current position value that control is based on.
    virtual units::meter_t GetPosition() = 0;
};

class VelocityMotorTuner : virtual public BaseMotorTuner {
public:
    VelocityMotorTuner( std::string_view name, tuning::Parameters p, tuning::ControlType ctrl );

        // Update and provide the velocity to use for control.
    void Update( units::revolutions_per_minute_t velocity, double arbFF = 0.0 );

        // Update and use the default encoder for velocity control.
    void Update( double arbFF = 0.0 );

    virtual void SetVelocity( units::revolutions_per_minute_t goal );

protected:
    units::revolutions_per_minute_t m_Goal;

        // Software Control Stuff
    frc::SimpleMotorFeedforward<units::turns> *m_simpleFF;

    void CreateSoftwareFeedForward();

        // Must override to provide current velocity value that control is based on.
    virtual units::revolutions_per_minute_t GetVelocity() = 0;
};

}  // namespace tuning