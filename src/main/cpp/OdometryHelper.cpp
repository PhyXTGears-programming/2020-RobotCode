#include "OdometryHelper.h"

#include <iostream>

#include <AHRS.h>
#include <frc/SPI.h>

#define PI 3.14159265358979323846

constexpr units::length::inch_t kHalfWheelBase{(23.0 + 7.0/8.0) / 2.0};
constexpr double kWheelRadiansPerMotorRotation = (1 / 10.71) * (2 * PI); // Encoder ticks per radian

AHRS navx {frc::SPI::Port::kMXP}; // Declared here because the AHRS.h file contains "using namespace std;"

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
    m_Odometry = new frc::DifferentialDriveOdometry(GetRobotAngle(), robotInitialPostion);
}

int c = 0;
void OdometryHelper::Update () {
    if (c % 50 == 0) {
        std::cout << "Angular Position: " << GetAngularPosition(WheelSide::leftWheels) << " " << GetAngularPosition(WheelSide::rightWheels) << std::endl;
        std::cout << "Angular Velocity: " << GetAngularVelocity(WheelSide::leftWheels) << " " << GetAngularVelocity(WheelSide::rightWheels) << std::endl;
        std::cout << "Linear Position: " << GetLinearPosition(WheelSide::leftWheels) << " " << GetLinearPosition(WheelSide::rightWheels) << std::endl;
        std::cout << "Linear Velocity: " << GetLinearVelocity(WheelSide::leftWheels) << " " << GetLinearVelocity(WheelSide::rightWheels) << std::endl;
        std::cout << "Angle: " << GetRobotAngle().Degrees() << std::endl;
    }
    m_Odometry->Update(GetRobotAngle(), GetLinearPosition(WheelSide::leftWheels), GetLinearPosition(WheelSide::rightWheels));
    c++;
}

// GetRobotPose

frc::Rotation2d OdometryHelper::GetRobotAngle () {
    return frc::Rotation2d(units::angle::degree_t(-navx.GetYaw()));
}

rev::CANEncoder* OdometryHelper::GetEncoder (WheelSide side) {
    switch (side) {
        case WheelSide::leftWheels:
            return m_LeftEncoder;
        case WheelSide::rightWheels:
            return m_RightEncoder;
        default:
            return nullptr;
    }
}
