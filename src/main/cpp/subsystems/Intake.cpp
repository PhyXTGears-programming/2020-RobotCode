#include "subsystems/Intake.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>

Intake::Intake () {}

void Intake::Periodic () {}

void Intake::SetSpeed (double intakeSpeed) {
    m_IntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, intakeSpeed);
}

void Intake::IntakeStart () {
    SetSpeed(1.0);
}

void Intake::IntakeStop () {
    SetSpeed(0.0);
}

void Intake::IntakeReverse () {
    SetSpeed(-0.5);
}
