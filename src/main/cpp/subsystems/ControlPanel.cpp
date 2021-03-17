#include "subsystems/ControlPanel.h"

#include <units/length.h>

using namespace units::literals;

#define kMotorTurnsPerWheelTurn 10.0
#define kWheelDiameter 3_in

#define kWheelTurnsPerPanelTurn 

ControlPanel::ControlPanel (std::shared_ptr<cpptoml::table> toml) {
    config.extraDegrees  = toml->get_qualified_as<double>("extraDegrees").value_or(0.0);
    config.extraDegrees  = toml->get_qualified_as<double>("p").value_or(0.0);

    m_RotationMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_RotationMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
    m_RotationMotor.Config_kP(0, config.p);
    m_RotationMotor.Config_kI(0, 0);
    m_RotationMotor.Config_kD(0, 0);
    m_RotationMotor.Config_kF(0, 0);
}

void ControlPanel::Rotate (units::angle::turn_t angle) {
    units::angle::turn_t motorTurns = angle * (32_in / kWheelDiameter) * kMotorTurnsPerWheelTurn;
    double sensorUnits = units::unit_cast<double>(motorTurns) * 4096;

    double target = m_RotationMotor.GetSelectedSensorPosition() + sensorUnits;

    m_RotationMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, target);
}

void ControlPanel::Position (ControlPanelColor color) {
    // TODO
}
