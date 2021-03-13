#pragma once

#include <cpptoml.h>
#include <units/units.h>

#include <frc/SpeedControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"

enum class TrackingMode { Off, GyroTracking, CameraTracking, Auto };

class Shooter : public frc2::SubsystemBase {
    public:
        Shooter(std::shared_ptr<cpptoml::table> toml);
        void Periodic() override;

        void SetShooterMotorSpeed(units::angular_velocity::revolutions_per_minute_t speed);
        units::angular_velocity::revolutions_per_minute_t GetShooterMotorSpeed();

        void SetTrackingMode(TrackingMode mode);

        void SetTurretSpeed(units::angular_velocity::revolutions_per_minute_t speed);
        void SetTurretSpeed(double percentSpeed);
        
        units::angular_velocity::revolutions_per_minute_t GetShooterSpeedForDistance();

        int GetTargetCount();

        bool IsOnTarget();

        double MeasureShooterMotorSpeed1();
        double MeasureShooterMotorSpeed2();

        void SetLimelightLight(bool on);

    private:
        void TrackingPeriodic(TrackingMode mode);

        TrackingMode m_TrackingMode = TrackingMode::Off;

        int m_TargetCount = 0;
        double m_TargetErrorX = 0.0;
        double m_TargetErrorY = 0.0;

        rev::CANSparkMax m_ShooterMotor1 {kShooterMotor1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_ShooterMotor2 {kShooterMotor2, rev::CANSparkMax::MotorType::kBrushless};

        ctre::phoenix::motorcontrol::can::TalonSRX m_TurretMotor {kTurretMotor};

        std::shared_ptr<nt::NetworkTable> m_VisionTable;
        
        frc2::PIDController* m_TurretPID;
        
        struct {
            struct {
                double p, i, d, f;
            } turretVelocity;

            struct {
                double p, i, d;
            } turretPosition;

            struct {
                double p, i, d, f;
            } shooterVelocity;

            struct {
                units::angular_velocity::revolutions_per_minute_t near, far;
            } shootingSpeed;
        } config;
};
