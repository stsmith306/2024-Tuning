#pragma once

#include <string>

#include "AbsoluteEncoder.h"

#include <ctre/phoenix6/CANcoder.hpp>


class CTRECANcoder : public AbsoluteEncoder {
public:
    CTRECANcoder( std::string_view pref_name, int CAN_Id, std::string canbus="" ) 
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

    void ResetOffset( ) {
        ctre::phoenix6::configs::CANcoderConfiguration configs{};
        m_Encoder.GetConfigurator().Refresh(configs);        
        configs.MagnetSensor.MagnetOffset = m_offset.value();
        m_Encoder.GetConfigurator().Apply(configs);        
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
