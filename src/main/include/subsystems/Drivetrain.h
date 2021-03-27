#pragma once

#include <AHRS.h>
#include <cpptoml.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/current.h>

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain(std::shared_ptr<cpptoml::table> toml);

        void Periodic() override;

        void Drive(double yInput, double xInput);
        void RadiusDrive(double speed, double radius);

        void SetBrake(bool on);

        void SetAcceleration (double a) { SetDrivingMode(false); acceleration = a; }
        void SetAcceleration (double a, double target) { SetDrivingMode(false); acceleration = a; targetSpeed = target; }
        void SetAngularVelocity (double w) { SetDrivingMode(false); angularVelocity = w; }
        void SetAngularVelocity (double w, double target) { SetDrivingMode(false); angularVelocity = w; targetAngle = target; }

        frc::Pose2d GetPose () { return odometry.GetPose(); }
        void SetPose(double x, double y, double angle = 0);
        void ResetPose (double angle = 0) { SetPose(0, 0, angle); }

        double GetSpeed () { return (leftLeader.GetEncoder().GetVelocity() + rightLeader.GetEncoder().GetVelocity()) / 2.0; }

        double GetVoltage () { return (leftLeader.GetBusVoltage() + rightLeader.GetBusVoltage()) / 2.0; }
        double GetMaxAvailableAcceleration () { return (GetVoltage() - voltageUsedWithoutAcceleration) / config.kinematics.ka; }

        double GetKS () { return config.kinematics.ks; }
        double GetKV () { return config.kinematics.kv; }
        double GetKA () { return config.kinematics.ka; }
        double GetKW () { return config.kinematics.kw; }

        units::current::ampere_t GetMotorCurrent () {
            return units::current::ampere_t{(leftLeader.GetOutputCurrent() + rightLeader.GetOutputCurrent()) / 2.0};
        }

    private:
        void UpdateOdometry();
        void UpdateVoltages();

        double GetLinearVoltage();
        double GetRotationalVoltage();
        double GetAccelerationCorrection(double speed);
        double GetRotationalCorrection();

        void SetDrivingMode (bool old) { SetBrake(false); oldDriving = old; }
        
        struct {
            struct {
                double ks, kv, ka, kw;
            } kinematics;
        } config;

        bool brakeOn;
        bool oldDriving = true;

        double acceleration = 0;
        double angularVelocity = 0;
        double targetSpeed = 0;
        double targetAngle = 0;

        double gyroZeroAngle;

        double voltageUsedWithoutAcceleration = 0;

        rev::CANSparkMax leftLeader {DriveMotorPins::Left1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax leftFollower1 {DriveMotorPins::Left2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax leftFollower2 {DriveMotorPins::Left3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax rightLeader {DriveMotorPins::Right1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax rightFollower1 {DriveMotorPins::Right2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax rightFollower2 {DriveMotorPins::Right3, rev::CANSparkMax::MotorType::kBrushless};

        frc::SpeedControllerGroup leftGroup {leftLeader, leftFollower1, leftFollower2};
        frc::SpeedControllerGroup rightGroup {rightLeader, rightFollower1, rightFollower2};

        AHRS gyro {frc::SPI::Port::kMXP};

        frc::DifferentialDriveOdometry odometry {gyro.GetRotation2d()};
};
