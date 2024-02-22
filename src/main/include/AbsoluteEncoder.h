#pragma once

#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>

// Uses the DutyCycleEncoder class to use an SRX mag absolute encoder
class AbsoluteEncoder {
    public:
        // Offset must in the range from 0 to 1
        AbsoluteEncoder( const int absoluteEncoderChannel, const double absoluteEncoderOffset = 0.0) : 
                        m_absoluteEncoder{absoluteEncoderChannel}, 
                        m_absoluteEncoderOffset{absoluteEncoderOffset} { }

        // Returns the angle in degrees
        units::degree_t GetAbsolutePosition( void ) {
            return ( m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoderOffset ) * 360_deg;
        }

        units::degree_t GetDistance() {
            return (m_absoluteEncoder.Get() - m_absoluteEncoderOffset * 1_tr) + m_rollover_offset;
        }

        double GetRawPosition( ) {
            return m_absoluteEncoder.GetAbsolutePosition();
        }

        bool IsConnected( void ) {
            return m_absoluteEncoder.IsConnected();
        }

        void SetPositionOffset() {

            if( GetDistance() < -180_deg ) {
                m_rollover_offset = 1_tr;
            }

            double offset_delta = m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoderOffset;
            fmt::print("Absolute Position {}\n", m_absoluteEncoder.GetAbsolutePosition());
            fmt::print("Absolute Offset {}\n", m_absoluteEncoderOffset);
            fmt::print("Distance {}\n", GetDistance() );

        }

    private:
        frc::DutyCycleEncoder m_absoluteEncoder;

        const double m_absoluteEncoderOffset;
        units::turn_t m_rollover_offset{0};
};