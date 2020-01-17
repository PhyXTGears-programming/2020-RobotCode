#include "subsystems/Shooter.h"
#include "Robot.h"

#include <iostream>

#define SetPID(motor, P, I, D) SetPIDSlot(motor, P, I, D, 0)
#define SetPIDSlot(motor, P, I, D, slot) motor.SetP(P, slot); motor.SetI(I, slot); motor.SetD(D, slot);

Shooter::Shooter () {
    // Implementation of subsystem constructor goes here.
    SetPID(m_ShooterMotor1PID, 1.0, 0.0, 0.0)
    SetPID(m_ShooterMotor2PID, 1.0, 0.0, 0.0)
}
     
void Shooter::Periodic () {
    // Implementation of subsystem periodic method goes here.
}

void Shooter::SetShooterMotorSpeeds (double speed1, double speed2) {
    m_ShooterMotor1PID.SetReference(speed1, rev::kVelocity);
    m_ShooterMotor2PID.SetReference(speed2, rev::kVelocity);
}
