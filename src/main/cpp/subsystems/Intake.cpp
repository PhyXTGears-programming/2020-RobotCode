#include "subsystems/Intake.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>

#define kCornerMotorAdjust 1

Intake::Intake () {
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
    SetIntakeSpeed(-1);
}

void Intake::IntakeStop () {
    SetIntakeSpeed(0);
}

void Intake::IntakeReverse () {
    SetIntakeSpeed(0.5);
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

bool Intake::IsPowerCellInFeeder() {
    return m_FeederPowerCellDetector.Get();
}