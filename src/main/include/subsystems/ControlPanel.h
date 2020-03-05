#pragma once

#include <cpptoml.h>
#include <units/units.h>

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

#include "Constants.h"

enum class ControlPanelColor { Red, Yellow, Green, Blue };

class ControlPanel : public frc2::SubsystemBase {
    public:
        ControlPanel(std::shared_ptr<cpptoml::table> toml);
        void Periodic () override {}

        void Rotate () { Rotate(units::angle::degree_t{1080 + config.extraDegrees}); }
        void Rotate(units::angle::turn_t angle);

        void Position(ControlPanelColor color);

        void SetSpeed (double speed) {
            m_RotationMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
        }

        void Extend () {
            m_ExtendSolenoid.Set(true);
            m_RetractSolenoid.Set(false);
        }

        void Retract () {
            m_ExtendSolenoid.Set(false);
            m_RetractSolenoid.Set(true);
        }

    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_RotationMotor {kIntakeMotor};

        frc::Solenoid m_ExtendSolenoid {kControlPanelExtendSolenoidPin};
        frc::Solenoid m_RetractSolenoid {kControlPanelRetractSolenoidPin};
        
        struct {
            double extraDegrees;
            double p;
        } config;
};

