#include "subsystems/Drivetrain.h"

#include <iostream>
#include <math.h>

#include "Robot.h"

#define PI 3.14159265358979323846

#define kTurnInputConstant 0.3 // lowering makes robot drive more straight, raising makes it turn more at any given input (other than -1, 0, or 1)

#define kHalfWheelBase 0.953125

constexpr auto kWheelDiameter = 6_in;
constexpr double kWheelRadiansPerMotorRotation = (1 / 10.71) * (2 * PI); // Encoder ticks per radian
constexpr auto kDistancePerWheelRadian = (kWheelDiameter/2) / (1_rad);

#define leftSideAngularPosition()  units::angle::radian_t(m_LeftMotor1.GetEncoder().GetPosition())
#define rightSideAngularPosition() units::angle::radian_t(m_RightMotor1.GetEncoder().GetPosition())

#define leftSidePosition()  leftSideAngularPosition() * kDistancePerWheelRadian
#define rightSidePosition() rightSideAngularPosition() * kDistancePerWheelRadian

#define leftSideAngularVelocity()  units::angular_velocity::radians_per_second_t(m_LeftMotor1.GetEncoder().GetVelocity())
#define rightSideAngularVelocity() units::angular_velocity::radians_per_second_t(m_RightMotor1.GetEncoder().GetVelocity())

#define leftSideVelocity()  leftSideAngularVelocity() * kDistancePerWheelRadian
#define rightSideVelocity() rightSideAngularVelocity() * kDistancePerWheelRadian

Drivetrain::Drivetrain () {
    // Position in wheel angular displacement (rad)
    m_LeftMotor1.GetEncoder().SetPositionConversionFactor(kWheelRadiansPerMotorRotation);
    m_RightMotor1.GetEncoder().SetPositionConversionFactor(kWheelRadiansPerMotorRotation);

    // Velocity in wheel angular velocity (rad/s)
    m_LeftMotor1.GetEncoder().SetVelocityConversionFactor(kWheelRadiansPerMotorRotation / 60.0);
    m_RightMotor1.GetEncoder().SetVelocityConversionFactor(kWheelRadiansPerMotorRotation / 60.0);

    // Initial Position is 0
    m_LeftMotor1.GetEncoder().SetPosition(0.0);
    m_RightMotor1.GetEncoder().SetPosition(0.0);

    frc::Rotation2d gyroAngle {units::angle::degree_t(-1.0)}; // replace with gyro angle
    frc::Pose2d robotInitialPostion {1_ft, 1_ft, 1_rad}; // replace with robot inital coordinates and angle
    m_Odometry = new frc::DifferentialDriveOdometry(gyroAngle, robotInitialPostion);

    // Brake defaults to on
    SetBrake(true);
}

void Drivetrain::Periodic () {
    frc::Rotation2d gyroAngle {units::angle::degree_t(-1.0)}; // replace with gyro angle
    m_Odometry->Update(gyroAngle, leftSidePosition(), rightSidePosition());
}

// Calculate radius from x stick, and drive
void Drivetrain::Drive (double yInput, double xInput) {
    // Radius math in Desmos (https://www.desmos.com/calculator/htvtwcp39g)
    xInput = pow(xInput, 3);
    double r = 1/(kTurnInputConstant * xInput) - xInput/kTurnInputConstant;

    // When the radius is 0, RadiusDrive turns the robot clockwise by default
    // When the x stick is all the way left, the radius is 0, but the robot should turn counterclockwise
    // By setting the radius to -0, it will turn counterclockwise
    if (xInput == -1) {
        r = -0.0;
    }

    RadiusDrive(yInput, r);
}

// Given a radius and speed, drive around the circle
void Drivetrain::RadiusDrive (double speed, double radius) {
    // Reverse turning direction when driving backwards.  Special request by Caleb S.
    radius *= std::copysign(1.0, speed);

    // Calculate the radius for each wheel
    double leftWheelRadius = radius + kHalfWheelBase;
    double rightWheelRadius = radius - kHalfWheelBase;

    // Default speed is 1
    double leftWheelSpeed = 1;
    double rightWheelSpeed = 1;

    /*
     * Scale the slower wheel (the inside wheel) to a proportion of the faster wheel speed (left at 1)
     * Because both circles need to be traveled by their corresponding wheel in the same amount of time, speed is calculated as:
     * (smaller circumference / larger circumference) * faster wheel speed
     * where faster wheel speed is always 1
     * Since circumference is proportional to radius, radius is used instead
     */
    if (isinf(radius)) {
        // Do nothing, when driving straight, both wheels drive at 1
    } else if (radius > 0) { // Right wheel has smaller radius and goes slower
        rightWheelSpeed = fabs(rightWheelRadius) / fabs(leftWheelRadius);
    } else if (radius < 0) { // Left wheel has smaller radius and goes slower
        leftWheelSpeed = fabs(leftWheelRadius) / fabs(rightWheelRadius);
    }

    // Make one wheel reverse when radius is inside the wheelbase
    leftWheelSpeed *= std::copysign(1.0, leftWheelRadius) * std::copysign(1.0, radius);
    rightWheelSpeed *= std::copysign(1.0, rightWheelRadius) * std::copysign(1.0, radius);

    // Scale speed based on speed input
    leftWheelSpeed *= speed;
    rightWheelSpeed *= -speed; // Negative because right wheels are mounted backwards (one side is always backwards)

    // Write to motors
    m_LeftMotors.Set(leftWheelSpeed);
    m_RightMotors.Set(rightWheelSpeed);
}

void Drivetrain::SetBrake (bool on) {
    m_LeftMotor1.SetIdleMode(on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    m_LeftMotor2.SetIdleMode(on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    m_LeftMotor3.SetIdleMode(on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    m_RightMotor1.SetIdleMode(on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    m_RightMotor2.SetIdleMode(on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    m_RightMotor3.SetIdleMode(on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
}
