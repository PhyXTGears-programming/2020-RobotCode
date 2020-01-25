#include "subsystems/Intake.h"


Intake::Intake() {
    // Implementation of subsystem constructor goes here.
}

void Intake::Periodic() {
    // Implementation of subsystem periodic method goes here.
}

void Intake::SetSpeed(double intakeSpeed) {
    m_IntakeMotor.Set(ControlMode::PercentOutput, intakeSpeed);
}
void IntakeStart(){
    //activate intake motor
    setSpeed(1.0);
}

void IntakeStop(){
    //stop intake motor
    setSpeed(0.0);
}

void IntakeReverse(){
    //reverse intake motor and run
    setSpeed(-0.5);
}

