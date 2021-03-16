#pragma once

#include <AHRS.h>

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/length.h>
#include <units/current.h>

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain();

        void Periodic() override;

        void Drive(double yInput, double xInput);
        void RadiusDrive(double speed, double radius);

        void SetBrake(bool on);

        void SetAcceleration (double a) { oldDriving = false; acceleration = a; }
        void SetAcceleration (double a, double target) { oldDriving = false; acceleration = a; targetSpeed = target; }
        void SetAngularVelocity (double w) { oldDriving = false; angularVelocity = w; }
        void SetAngularVelocity (double w, double target) { oldDriving = false; angularVelocity = w; targetAngle = target; }

        void ResetPose(double angle = 0);

        frc::Pose2d GetPose () { return odometry.GetPose(); }
        double GetSpeed () { return (leftLeader.GetEncoder().GetVelocity() + rightLeader.GetEncoder().GetVelocity()) / 2.0; }

        double GetVoltage () { return (leftLeader.GetBusVoltage() + rightLeader.GetBusVoltage()) / 2.0; }
        double GetMaxAvailableAcceleration () { return (GetVoltage() - voltageUsedWithoutAcceleration) / Ka; }

        double GetKS () { return Ks; }
        double GetKV () { return Kv; }
        double GetKA () { return Ka; }
        double GetKW () { return Kw; }

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
        
        double Ks, Kv, Ka, Kw;

        bool brakeOn;
        bool oldDriving = true;

        double acceleration = 0;
        double angularVelocity = 0;
        double targetSpeed = 0;
        double targetAngle = 0;

        double gyroZeroAngle;

        double voltageUsedWithoutAcceleration = 0;

        static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
        static constexpr double kWheelRadius = 0.0508; // meters
        static constexpr int kEncoderResolution = 4096;

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
