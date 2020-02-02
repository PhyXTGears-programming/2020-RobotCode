#include "OdometryHelper.h"

#include <iostream>

#define PI 3.14159265358979323846

constexpr units::length::inch_t kHalfWheelBase{(23.0 + 7.0/8.0) / 2.0};
constexpr double kWheelRadiansPerMotorRotation = (1 / 10.71) * (2 * PI); // Encoder ticks per radian

// #define leftSideAngularPosition()  units::angle::radian_t(m_LeftEncoder.GetPosition())
// #define rightSideAngularPosition() units::angle::radian_t(m_RightEncoder.GetPosition())

// #define leftSidePosition()  leftSideAngularPosition() * kDistancePerWheelRadian
// #define rightSidePosition() rightSideAngularPosition() * kDistancePerWheelRadian

// #define leftSideAngularVelocity()  units::angular_velocity::radians_per_second_t(m_LeftEncoder.GetVelocity())
// #define rightSideAngularVelocity() units::angular_velocity::radians_per_second_t(m_RightEncoder.GetVelocity())

// #define leftSideVelocity()  leftSideAngularVelocity() * kDistancePerWheelRadian
// #define rightSideVelocity() rightSideAngularVelocity() * kDistancePerWheelRadian

OdometryHelper::OdometryHelper (rev::CANEncoder* leftEncoder, rev::CANEncoder* rightEncoder) {
    m_LeftEncoder = leftEncoder;
    m_RightEncoder = rightEncoder;

    // Position in wheel angular displacement (rad)
    m_LeftEncoder->SetPositionConversionFactor(kWheelRadiansPerMotorRotation);
    m_RightEncoder->SetPositionConversionFactor(kWheelRadiansPerMotorRotation);

    // Velocity in wheel angular velocity (rad/s)
    m_LeftEncoder->SetVelocityConversionFactor(kWheelRadiansPerMotorRotation / 60.0);
    m_RightEncoder->SetVelocityConversionFactor(kWheelRadiansPerMotorRotation / 60.0);

    // Initial Position is 0
    m_LeftEncoder->SetPosition(0.0);
    m_RightEncoder->SetPosition(0.0);

    frc::Pose2d robotInitialPostion {0_ft, 0_ft, 0_rad}; // replace with robot inital coordinates and angle
    // m_Odometry = new frc::DifferentialDriveOdometry(GetGyroAngle(), robotInitialPostion);
}

void OdometryHelper::Update () {
    // std::cout << "Angular Position: " << GetAngularPosition(WheelSide::leftWheels) << " " << GetAngularPosition(WheelSide::rightWheels) << std::endl;
    // std::cout << "Angular Velocity: " << GetAngularVelocity(WheelSide::leftWheels) << " " << GetAngularVelocity(WheelSide::rightWheels) << std::endl;
    // std::cout << "Linear Position: " << GetLinearPosition(WheelSide::leftWheels) << " " << GetLinearPosition(WheelSide::rightWheels) << std::endl;
    // std::cout << "Linear Velocity: " << GetLinearVelocity(WheelSide::leftWheels) << " " << GetLinearVelocity(WheelSide::rightWheels) << std::endl;
    // m_Odometry->Update(GetGyroAngle(), GetLinearPosition(WheelSide::leftWheels), GetLinearPosition(WheelSide::rightWheels));
}

// rev::CANEncoder* OdometryHelper::GetEncoder (WheelSide side) {
//     switch (side) {
//         case WheelSide::leftWheels:
//             return m_LeftEncoder;
//         case WheelSide::rightWheels:
//             return m_RightEncoder;
//         default:
//             return nullptr;
//     }
// }
