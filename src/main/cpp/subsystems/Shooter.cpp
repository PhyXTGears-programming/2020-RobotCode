#include "subsystems/Shooter.h"
#include "Robot.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <iostream>

#define SetPID(motor, P, I, D) SetPIDSlot(motor, P, I, D, 0)
#define SetPIDSlot(motor, P, I, D, slot) motor.SetP(P, slot); motor.SetI(I, slot); motor.SetD(D, slot)

Shooter::Shooter () {
    // Implementation of subsystem constructor goes here.
    SetPID(m_ShooterMotor1PID, 0.00030, 0.0, 0.05);
    SetPID(m_ShooterMotor2PID, 0.00030, 0.0, 0.05);
}
     
void Shooter::Periodic () {
    // Implementation of subsystem periodic method goes here.
}

void Shooter::SetShooterMotorSpeeds (double speed1, double speed2) {
    // std::cout << "Shooter speeds: " << speed1 << " " << speed2 << std::endl;
    static double deltaTime = 0;
    static hal::fpga_clock::time_point now, timePrev, startTime = hal::fpga_clock::now();

    now = hal::fpga_clock::now();
    deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count() / 1000000.0;
    timePrev = now;
    std::cout << deltaTime << ", " << -m_ShooterMotor1Encoder.GetVelocity() << ", " << -m_ShooterMotor1Encoder.GetVelocity() << std::endl;

    if (speed1 == 0) {
        if (m_ShooterMotor1Encoder.GetVelocity() > 100) {
            m_ShooterMotor1.Set(-0.03);
        } else {
            m_ShooterMotor1.Set(0);
        }
    } else {
        m_ShooterMotor1PID.SetReference(speed1, rev::kVelocity);
    }
    if (speed2 == 0) {
        if (m_ShooterMotor2Encoder.GetVelocity() < -100) {
            m_ShooterMotor2.Set(0.03);
        } else {
            m_ShooterMotor2.Set(0);
        }
    } else {
        m_ShooterMotor2PID.SetReference(speed2, rev::kVelocity);
    }

    frc::SmartDashboard::PutNumber("Shooter Motor 1", m_ShooterMotor1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Motor 2", m_ShooterMotor2Encoder.GetVelocity());
}
