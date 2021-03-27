#include "subsystems/Shooter.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Robot.h"

#define SetPIDF(motor, vals) SetPIDFSlot(motor, vals.p, vals.i, vals.d, vals.f, 0)
#define SetPIDFSlot(motor, P, I, D, F, slot) motor.SetP(P, slot); motor.SetI(I, slot); motor.SetD(D, slot); motor.SetFF(F, slot)

#define kShooterGearRatio (18.0 / 24.0)

#define kTurretGearRatio (20.0 / 3.0)
#define kMotorRPMtoEncoderVelocity (4096 / (600_rpm)) // encoder velocity is measured in ticks per 100 ms

#define kMaxTurretVelocity 20_rpm

Shooter::Shooter (std::shared_ptr<cpptoml::table> toml) {
    config.turretVelocity.p = toml->get_qualified_as<double>("turretVelocity.p").value_or(0.0);
    config.turretVelocity.i = toml->get_qualified_as<double>("turretVelocity.i").value_or(0.0);
    config.turretVelocity.d = toml->get_qualified_as<double>("turretVelocity.d").value_or(0.0);
    config.turretVelocity.f = toml->get_qualified_as<double>("turretVelocity.f").value_or(0.0);

    config.turretPosition.p = toml->get_qualified_as<double>("turretPosition.p").value_or(0.0);
    config.turretPosition.i = toml->get_qualified_as<double>("turretPosition.i").value_or(0.0);
    config.turretPosition.d = toml->get_qualified_as<double>("turretPosition.d").value_or(0.0);
    
    config.shooterVelocity.p = toml->get_qualified_as<double>("shooterVelocity.p").value_or(0.0);
    config.shooterVelocity.i = toml->get_qualified_as<double>("shooterVelocity.i").value_or(0.0);
    config.shooterVelocity.d = toml->get_qualified_as<double>("shooterVelocity.d").value_or(0.0);
    config.shooterVelocity.f = toml->get_qualified_as<double>("shooterVelocity.f").value_or(0.0);

    config.shootingSpeed.near = units::angular_velocity::revolutions_per_minute_t{toml->get_qualified_as<double>("shootingSpeed.near").value_or(0.0)};
    config.shootingSpeed.far  = units::angular_velocity::revolutions_per_minute_t{toml->get_qualified_as<double>("shootingSpeed.far").value_or(0.0)};

    // Setup shooter motors
    SetPIDF(m_ShooterMotor1.GetPIDController(), config.shooterVelocity);
    SetPIDF(m_ShooterMotor2.GetPIDController(), config.shooterVelocity);

    m_ShooterMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_ShooterMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_ShooterMotor1.SetInverted(false);
    m_ShooterMotor2.SetInverted(true);

    // Setup vision NT
    m_VisionTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight-gears");

    // Set up turret motor velocity PIDF
    ctre::phoenix::motorcontrol::can::TalonSRXPIDSetConfiguration turretMotorPIDConfig {ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative};
    m_TurretMotor.ConfigurePID(turretMotorPIDConfig);
    m_TurretMotor.SetInverted(true);
    m_TurretMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_TurretMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
    m_TurretMotor.SetSensorPhase(true);
    m_TurretMotor.Config_kP(0, config.turretVelocity.p);
    m_TurretMotor.Config_kI(0, config.turretVelocity.i);
    m_TurretMotor.Config_kD(0, config.turretVelocity.d);
    m_TurretMotor.Config_kF(0, config.turretVelocity.f);

    // Set up turret motor position PID
    m_TurretPID = new frc2::PIDController(config.turretPosition.p, config.turretPosition.i, config.turretPosition.d);

    frc::SmartDashboard::PutNumber("Shooter P", config.shooterVelocity.p);
    frc::SmartDashboard::PutNumber("Shooter D", config.shooterVelocity.d);
    frc::SmartDashboard::PutNumber("Shooter F", config.shooterVelocity.f);

    // Light should start off
    SetLimelightLight(false);
}

void Shooter::Periodic () {
    // double factor = 1 / kShooterGearRatio;

    // frc::SmartDashboard::PutNumber("Shooter Motor 1", m_ShooterMotor1.GetEncoder().GetVelocity() * factor);
    // frc::SmartDashboard::PutNumber("Shooter Motor 2", m_ShooterMotor2.GetEncoder().GetVelocity() * factor);
    
    // frc::SmartDashboard::PutNumber("Turret Speed Read (RPM)", units::unit_cast<double>(m_TurretMotor.GetSelectedSensorVelocity() / kTurretGearRatio / kMotorRPMtoEncoderVelocity));

    // config.shooterVelocity.p = frc::SmartDashboard::GetNumber("Shooter P", config.shooterVelocity.p);
    // config.shooterVelocity.d = frc::SmartDashboard::GetNumber("Shooter D", config.shooterVelocity.d);
    // config.shooterVelocity.f = frc::SmartDashboard::GetNumber("Shooter F", config.shooterVelocity.f);

    // SetPIDF(m_ShooterMotor1.GetPIDController(), config.shooterVelocity);
    // SetPIDF(m_ShooterMotor2.GetPIDController(), config.shooterVelocity);

    TrackingPeriodic(m_TrackingMode);
}

void Shooter::SetShooterMotorSpeed (units::angular_velocity::revolutions_per_minute_t speed) {
    if (units::math::fabs(speed) > 50_rpm) {
        double motorSpeed = kShooterGearRatio * units::unit_cast<double>(speed);
        m_ShooterMotor1.GetPIDController().SetReference(motorSpeed, rev::kVelocity);
        m_ShooterMotor2.GetPIDController().SetReference(motorSpeed, rev::kVelocity);
    } else {
        m_ShooterMotor1.Set(0);
        m_ShooterMotor2.Set(0);
    }
}

units::angular_velocity::revolutions_per_minute_t Shooter::GetShooterMotorSpeed () {
    return units::angular_velocity::revolutions_per_minute_t(m_ShooterMotor1.GetEncoder().GetVelocity() / kShooterGearRatio);
}

void Shooter::SetTrackingMode (TrackingMode mode) {
    m_TrackingMode = mode;

    if (mode == TrackingMode::Off) {
        SetTurretSpeed(0_rpm);
    }
    
    if (mode == TrackingMode::CameraTracking) {
        SetLimelightLight(true);
    } else {
        SetLimelightLight(false);
    }
}

void Shooter::SetTurretSpeed (units::angular_velocity::revolutions_per_minute_t speed) {
    frc::SmartDashboard::PutNumber("Turret Speed Setpoint (RPM)", units::unit_cast<double>(speed));
    
    // std::cout << speed << " ";
    
    if (units::math::fabs(speed) < 1_deg_per_s) {
        m_TurretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        return;
    }

    double motorSpeed = kTurretGearRatio * units::unit_cast<double>(speed * kMotorRPMtoEncoderVelocity); // in encoder ticks per 100 ms
    
    // std::cout << motorSpeed << std::endl;

    m_TurretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, motorSpeed);
}

void Shooter::SetTurretSpeed (double percentSpeed) {
    SetTurretSpeed(percentSpeed * kMaxTurretVelocity);
}

units::angular_velocity::revolutions_per_minute_t Shooter::GetShooterSpeedForDistance () {
    auto const lo = -13.9;
    auto const hi = 3.2;

    double factor = (m_TargetErrorY - lo) / (hi - lo);

    // Clamp speed to far shooter speed.
    return config.shootingSpeed.near + (config.shootingSpeed.far - config.shootingSpeed.near) * std::clamp(factor, 0.0, 1.0);
}

int Shooter::GetTargetCount () {
    return m_TargetCount;
}

bool Shooter::IsOnTarget () {
    return 0 < m_TargetCount && 0.5 > std::fabs(m_TargetErrorX);
}

double Shooter::MeasureShooterMotorSpeed1 () {
    return m_ShooterMotor1.GetEncoder().GetVelocity() / kShooterGearRatio;
}

double Shooter::MeasureShooterMotorSpeed2 () {
    return m_ShooterMotor2.GetEncoder().GetVelocity() / kShooterGearRatio;
}

void Shooter::SetLimelightLight (bool on) {
    m_VisionTable->PutNumber("ledMode", on ? 3 : 1);
}

void Shooter::TrackingPeriodic (TrackingMode mode) {
    if (mode == TrackingMode::CameraTracking) {
        double speed = 0;

        m_TargetCount = (int) m_VisionTable->GetNumber("tv", -1);

        if (m_TargetCount > 0) {
            // std::cout << "Count: " << m_TargetCount << std::endl;
            frc::SmartDashboard::PutBoolean("Limelight Has Target", true);

            m_TargetErrorX = -m_VisionTable->GetNumber("tx", 0);
            m_TargetErrorY = -m_VisionTable->GetNumber("ty", 0);
            // std::cout << "m_TargetErrorX: " << m_TargetErrorX << std::endl;
            // std::cout << "m_TargetErrorY: " << m_TargetErrorY << std::endl;

            frc::SmartDashboard::PutNumber("Turret Error X", m_TargetErrorX);
            frc::SmartDashboard::PutNumber("Turret Error Y", m_TargetErrorY);

            speed = m_TurretPID->Calculate(m_TargetErrorX);
        } else if (m_TargetCount == 0) {
            // std::cout << "No objects detected" << std::endl;
            frc::SmartDashboard::PutBoolean("Limelight Has Target", false);
        } else {
            // std::cout << "Variable tv does not exist in table limelight-gears" << std::endl;
            frc::SmartDashboard::PutBoolean("Limelight Has Target", false);
        }

        SetTurretSpeed(speed);
    } else {
        frc::SmartDashboard::PutBoolean("Limelight Has Target", false);
        SetLimelightLight(false);
        return;
    }
}
