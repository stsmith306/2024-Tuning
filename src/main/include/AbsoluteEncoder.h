#pragma once

#include <string>

#include <units/angle.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Preferences.h>

// Base class for absolute encoders.  Uses the RoboRIO Preferences to store offsets.
class AbsoluteEncoder {
public:
    AbsoluteEncoder( std::string_view pref_name ) : m_pref_name{pref_name} {
        m_offset = frc::Preferences::GetDouble( m_pref_name, 0.0 ) * 1_tr;
    }

    void SetOffset( units::turn_t offset ) {
        m_offset = offset;
        frc::Preferences::SetDouble( m_pref_name, m_offset.value() );
    }

    virtual bool IsConnected( void ) = 0;
    virtual units::degree_t GetAbsolutePosition( ) = 0;
    virtual units::degree_t GetPosition( ) = 0;
    virtual units::revolutions_per_minute_t GetVelocity( ) = 0;
    virtual void SensorDirection( bool Clockwise_Positive ) = 0;
    virtual int GetChannel( void ) = 0;
protected:
    std::string m_pref_name;
    units::turn_t m_offset;
};

// Uses the DutyCycleEncoder class to use an SRX mag absolute encoder
class SRXMagEncoder : public AbsoluteEncoder {
public:
    // Offset must in the range from 0 to 1
    SRXMagEncoder( std::string_view pref_name, const int DIO_Channel ) : AbsoluteEncoder( pref_name ),
                    m_Encoder{DIO_Channel}
    {
          m_read_once = false; 

          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
          fmt::print(" SRXMagEncoder class IS INCOMPLETE!!!!  DO NOT USE !!!!!!!!!!!!\n");
    }

    // Returns the angle in degrees
    units::degree_t GetAbsolutePosition( void ) {
        return m_Encoder.GetAbsolutePosition() * 1_tr - m_offset;
    }

    units::degree_t GetPosition() {
        return (m_Encoder.Get() - m_offset);
    }

    bool IsConnected( void ) {
        return m_Encoder.IsConnected();
    }

    void SetPositionOffset() {

        if( GetPosition() < -180_deg ) {
            // m_rollover_offset = 1_tr;
        }

        // fmt::print("Absolute Position {}\n", m_absoluteEncoder.GetAbsolutePosition());
        // fmt::print("Absolute Offset {}\n", m_absoluteEncoderOffset);
        // fmt::print("Distance {}\n", GetDistance() );

    }

    int GetChannel( void ) { return m_Encoder.GetSourceChannel(); }

private:
    frc::DutyCycleEncoder m_Encoder;
    bool m_read_once;
};

class CTRECANCoder : public AbsoluteEncoder {
public:
    CTRECANCoder( std::string_view pref_name, int CAN_Id, std::string canbus="" ) 
        : AbsoluteEncoder( pref_name ), m_Encoder{ CAN_Id, canbus }
    {
        ctre::phoenix6::configs::CANcoderConfiguration configs{};
        configs.MagnetSensor.MagnetOffset = m_offset.value();
        m_Encoder.GetConfigurator().Apply(configs);
    }

    bool IsConnected( void ) {
        return !( m_Encoder.GetFault_BadMagnet().GetValue() ||
                  m_Encoder.GetFault_Hardware().GetValue() ||
                  m_Encoder.GetFault_Undervoltage().GetValue() ||
                  m_Encoder.GetFault_BootDuringEnable().GetValue() );
    }
    units::degree_t GetAbsolutePosition( ) { return m_Encoder.GetAbsolutePosition().GetValue(); }
    units::degree_t GetPosition( ) { return m_Encoder.GetPosition().GetValue(); }
    units::revolutions_per_minute_t GetVelocity( ) { return m_Encoder.GetVelocity().GetValue(); }
    void SensorDirection( bool Clockwise_Positive ) {
        ctre::phoenix6::configs::CANcoderConfiguration configs{};
        m_Encoder.GetConfigurator().Refresh(configs);
        configs.MagnetSensor.SensorDirection = Clockwise_Positive;
        m_Encoder.GetConfigurator().Apply(configs);
    }

    int GetChannel( void ) { return m_Encoder.GetDeviceID(); }

private:
    ctre::phoenix6::hardware::CANcoder m_Encoder;

};