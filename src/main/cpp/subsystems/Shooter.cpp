#include "subsystems/Shooter.h"
#include "Robot.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <iostream>

#define SetPID(motor, P, I, D) SetPIDSlot(motor, P, I, D, 0)
#define SetPIDF(motor, P, I, D, F) SetPIDFSlot(motor, P, I, D, F, 0)
#define SetPIDSlot(motor, P, I, D, slot) motor.SetP(P, slot); motor.SetI(I, slot); motor.SetD(D, slot)
#define SetPIDFSlot(motor, P, I, D, F, slot) motor.SetP(P, slot); motor.SetI(I, slot); motor.SetD(D, slot); motor.SetFF(F, slot)

double P = 0.00085;
double I = 0;
double D = 0.1;
double F = 0.00018;

Shooter::Shooter () {
    // Implementation of subsystem constructor goes here.
    SetPIDF(m_ShooterMotor1PID, P, I, D, F);
    SetPIDF(m_ShooterMotor2PID, P, I, D, F);

    m_ShooterMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_ShooterMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    frc::SmartDashboard::PutNumber("Shooter Motor F", F);
    frc::SmartDashboard::PutNumber("Shooter Motor P", P);
    frc::SmartDashboard::PutNumber("Shooter Motor D", D);
}
     
void Shooter::Periodic () {
    // Implementation of subsystem periodic method goes here.

    m_ShooterMotor1PID.SetFF(frc::SmartDashboard::GetNumber("Shooter Motor F", F));
    m_ShooterMotor2PID.SetFF(frc::SmartDashboard::GetNumber("Shooter Motor F", F));
    m_ShooterMotor1PID.SetP(frc::SmartDashboard::GetNumber("Shooter Motor P", P));
    m_ShooterMotor2PID.SetP(frc::SmartDashboard::GetNumber("Shooter Motor P", P));
    m_ShooterMotor1PID.SetD(frc::SmartDashboard::GetNumber("Shooter Motor D", D));
    m_ShooterMotor2PID.SetD(frc::SmartDashboard::GetNumber("Shooter Motor D", D));
}

void Shooter::SetShooterMotorSpeeds (double speed1, double speed2) {
    if (fabs(speed1) > 5) {
        m_ShooterMotor1PID.SetReference(speed1, rev::kVelocity);
    } else {
        m_ShooterMotor1.Set(0);
    }
    if (fabs(speed2) > 5) {
        m_ShooterMotor2PID.SetReference(speed2, rev::kVelocity);
    } else {
        m_ShooterMotor2.Set(0);
    }

    frc::SmartDashboard::PutNumber("Shooter Motor 1", m_ShooterMotor1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Motor 2", m_ShooterMotor2Encoder.GetVelocity());
}
