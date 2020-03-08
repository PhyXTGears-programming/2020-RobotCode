#include "subsystems/Intake.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>

Intake::Intake (std::shared_ptr<cpptoml::table> toml) {
    config.speed.load  = toml->get_qualified_as<double>("speed.load").value_or(0.0);
    config.speed.shoot = toml->get_qualified_as<double>("speed.shoot").value_or(0.0);

    m_ConveyorMotor.SetInverted(true);
}

void Intake::Periodic () {}

void Intake::SetIntakeSpeed (double intakeSpeed) {
    m_IntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, intakeSpeed);
}

void Intake::SetConveyorSpeed (double conveyorSpeed) {
    m_ConveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, conveyorSpeed);
}

void Intake::IntakeStart () {
    SetIntakeSpeed(-0.75);
}

void Intake::IntakeStop () {
    SetIntakeSpeed(0);
}

void Intake::IntakeReverse () {
    SetIntakeSpeed(0.75);
}

void Intake::ConveyorStart () {
    SetConveyorSpeed(1);
}

void Intake::ConveyorStop () {
    SetConveyorSpeed(0);
}

void Intake::ConveyorReverse () {
    SetConveyorSpeed(-0.5);
}

void Intake::IntakeExtend () {
    m_IntakeExtendSolenoid.Set(true);
    m_IntakeRetractSolenoid.Set(false);
    m_IsExtended = true;
}

void Intake::IntakeRetract () {
    m_IntakeExtendSolenoid.Set(false);
    m_IntakeRetractSolenoid.Set(true);
    m_IsExtended = false;
}

void Intake::FeedShooterStart () {
    SetFeederSpeed(config.speed.shoot);
}

void Intake::FeedLoadStart () {
    SetFeederSpeed(config.speed.load);
}

void Intake::FeedStop () {
    SetFeederSpeed(0.0);
}

bool Intake::IsExtended() {
    return m_IsExtended;
}

bool Intake::IsPowerCellInFeeder() {
    return m_FeederPowerCellDetector.Get();
}

void Intake::SetFeederSpeed (double percentSpeed) {
    m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -percentSpeed);
}
