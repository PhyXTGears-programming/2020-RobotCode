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

#define kShooterGearRatio (18.0 / 24.0)

#define kTurretGearRatio (20.0 / 3.0)
#define kMotorRPMtoEncoderVelocity (4096 / (600_rpm)) // encoder velocity is measured in ticks per 100 ms

#define kMaxTurretVelocity 20_rpm

Shooter::Shooter () {
    // Implementation of subsystem constructor goes here.
    SetPIDF(m_ShooterMotor1PID, P, I, D, F);
    SetPIDF(m_ShooterMotor2PID, P, I, D, F);

    m_ShooterMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_ShooterMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    frc::SmartDashboard::PutNumber("Shooter Motor F", F);
    frc::SmartDashboard::PutNumber("Shooter Motor P", P);
    frc::SmartDashboard::PutNumber("Shooter Motor D", D);

    m_VisionTable = nt::NetworkTableInstance::GetDefault().GetTable("Vision");

    // ctre::phoenix::motorcontrol::can::TalonSRXPIDSetConfiguration turretMotorPIDConfig {ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative};
    // m_TurretMotor.ConfigurePID(turretMotorPIDConfig);
    m_TurretMotor.SetInverted(true);
    m_TurretMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_TurretMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
    m_TurretMotor.SetSensorPhase(true);
    m_TurretMotor.Config_kP(0, 2.5);
    m_TurretMotor.Config_kI(0, 0.0);
    m_TurretMotor.Config_kD(0, 0.0);
    m_TurretMotor.Config_kF(0, 2.3);
}

void Shooter::Periodic () {
    // Implementation of subsystem periodic method goes here.
    m_ShooterMotor1PID.SetFF(frc::SmartDashboard::GetNumber("Shooter Motor F", F));
    m_ShooterMotor2PID.SetFF(frc::SmartDashboard::GetNumber("Shooter Motor F", F));
    m_ShooterMotor1PID.SetP(frc::SmartDashboard::GetNumber("Shooter Motor P", P));
    m_ShooterMotor2PID.SetP(frc::SmartDashboard::GetNumber("Shooter Motor P", P));
    m_ShooterMotor1PID.SetD(frc::SmartDashboard::GetNumber("Shooter Motor D", D)); 
    m_ShooterMotor2PID.SetD(frc::SmartDashboard::GetNumber("Shooter Motor D", D));

    double speed = 0;

    if (m_TrackingActive) {
        int count = (int) m_VisionTable->GetNumber("NumShapes", -1);

        if (count > 0) {
            std::cout << "Count: " << count << std::endl;

            double x = -m_VisionTable->GetNumber("Shape1x", 0);
            std::cout << "x: " << x << std::endl;

            speed = m_TurretPID.Calculate(x);
        } else if (count == 0) {
            std::cout << "No objects detected" << std::endl;
        } else {
            std::cout << "Variable NumShapes does not exist in table Vision" << std::endl;
        }
    }

    std::cout << speed << std::endl;
    SetTurretSpeed(kMaxTurretVelocity * speed);
}

void Shooter::SetShooterMotorSpeed (units::angular_velocity::revolutions_per_minute_t speed) {
    if (units::math::fabs(speed) > 50_rpm) {
        double motorSpeed = kShooterGearRatio * units::unit_cast<double>(speed);
        m_ShooterMotor1PID.SetReference(-motorSpeed, rev::kVelocity);
        m_ShooterMotor2PID.SetReference(motorSpeed, rev::kVelocity);
    } else {
        m_ShooterMotor1.Set(0);
        m_ShooterMotor2.Set(0);
    }

    frc::SmartDashboard::PutNumber("Shooter Motor 1", m_ShooterMotor1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Motor 2", m_ShooterMotor2Encoder.GetVelocity());
}

void Shooter::SetTurretSpeed (units::angular_velocity::revolutions_per_minute_t speed) {
    frc::SmartDashboard::PutNumber("Turret Speed Setpoint (RPM)", units::unit_cast<double>(speed));
    frc::SmartDashboard::PutNumber("Turret Speed Read (RPM)", units::unit_cast<double>(m_TurretMotor.GetSelectedSensorVelocity() / kTurretGearRatio / kMotorRPMtoEncoderVelocity));
    
    if (units::math::fabs(speed) < 1_deg_per_s) {
        m_TurretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        return;
    }

    double motorSpeed = kTurretGearRatio * units::unit_cast<double>(speed * kMotorRPMtoEncoderVelocity); // in encoder ticks per 100 ms
    
    m_TurretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, motorSpeed);
}
