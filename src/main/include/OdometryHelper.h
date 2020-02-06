#pragma once

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <rev/CANSparkMax.h>

constexpr auto kWheelDiameter = 5.75_in;
constexpr auto kDistancePerWheelRadian = (kWheelDiameter/2) / (1_rad);

enum class WheelSide {
    leftWheels,
    rightWheels
};

class OdometryHelper {
    public:
        OdometryHelper(rev::CANEncoder* leftEncoder, rev::CANEncoder* rightEncoder);

        void Update();

        inline int GetEncoderSign (WheelSide side) { return (side == WheelSide::leftWheels) * -2 + 1; }

        inline units::angle::radian_t GetAngularPosition (WheelSide side) {
            return GetEncoderSign(side) * units::angle::radian_t(GetEncoder(side)->GetPosition());
        }
        inline units::angular_velocity::radians_per_second_t GetAngularVelocity (WheelSide side) {
            return GetEncoderSign(side) * units::angular_velocity::radians_per_second_t(GetEncoder(side)->GetVelocity());
        }

        inline units::length::foot_t GetLinearPosition (WheelSide side) { return kDistancePerWheelRadian * GetAngularPosition(side); }
        inline units::velocity::meters_per_second_t GetLinearVelocity (WheelSide side) { return kDistancePerWheelRadian * GetAngularVelocity(side); }

        frc::Pose2d GetRobotPose();
        frc::Rotation2d GetRobotAngle();

    private:
        rev::CANEncoder* GetEncoder(WheelSide side);

        frc::DifferentialDriveOdometry* m_Odometry;
        rev::CANEncoder* m_LeftEncoder;
        rev::CANEncoder* m_RightEncoder;
};
