#pragma once

#include <cpptoml.h>
#include <units/angle.h>

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
            m_RetractSolenoid1.Set(false);
            m_RetractSolenoid2.Set(false);
            m_ExtendSolenoid1.Set(true);
            m_ExtendSolenoid2.Set(true);
        }

        void Retract () {
            m_ExtendSolenoid1.Set(false);
            m_ExtendSolenoid2.Set(false);
            m_RetractSolenoid1.Set(true);
            m_RetractSolenoid2.Set(true);
            SetSpeed(0);
        }

    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_RotationMotor {ControlPanelPins::kControlPanelMotor};

        frc::Solenoid m_ExtendSolenoid1 {ControlPanelPins::kControlPanelExtendSolenoid1Pin};
        frc::Solenoid m_ExtendSolenoid2 {ControlPanelPins::kControlPanelExtendSolenoid2Pin};
        frc::Solenoid m_RetractSolenoid1 {ControlPanelPins::kControlPanelRetractSolenoid1Pin};
        frc::Solenoid m_RetractSolenoid2 {ControlPanelPins::kControlPanelRetractSolenoid2Pin};
        
        struct {
            double extraDegrees;
            double p;
        } config;
};
