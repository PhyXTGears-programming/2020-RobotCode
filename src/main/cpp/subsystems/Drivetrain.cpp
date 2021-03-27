#include "subsystems/Drivetrain.h"

#include <iostream>
#include <math.h>
#include <algorithm>
#include <frc/RobotController.h>

#include "Robot.h"

#define PI 3.14159265358979323846

// lowering makes robot drive more straight, raising makes it turn more at any given input (other than -1, 0, or 1)
#define kTurnInputConstant 0.37

// half of the distance between the wheels in meters
#define kHalfWheelBase 0.953125

Drivetrain::Drivetrain (std::shared_ptr<cpptoml::table> toml) {
    config.kinematics.ks = toml->get_qualified_as<double>("kinematics.ks").value_or(0.0);
    config.kinematics.kv = toml->get_qualified_as<double>("kinematics.kv").value_or(0.0);
    config.kinematics.ka = toml->get_qualified_as<double>("kinematics.ka").value_or(0.0);
    config.kinematics.kw = toml->get_qualified_as<double>("kinematics.kw").value_or(0.0);

    static constexpr double metersPerMotorRotation = 0.04526269;

    for (auto m : {&leftLeader, &leftFollower1, &leftFollower2, &rightLeader, &rightFollower1, &rightFollower2}) {
        // Position in wheel angular displacement (rad)
        m->GetEncoder().SetPositionConversionFactor(metersPerMotorRotation);

        // Velocity in wheel angular velocity (rad/s)
        m->GetEncoder().SetVelocityConversionFactor(metersPerMotorRotation / 60);

        // Initial Position is 0
        m->GetEncoder().SetPosition(0);
    }

    rightLeader.SetInverted(true);
    rightFollower1.SetInverted(true);
    rightFollower2.SetInverted(true);

    ResetPose();

    // Brake defaults to on
    SetBrake(true);
}

void Drivetrain::Periodic () {
    UpdateOdometry();

    if (frc::RobotController::IsSysActive() && !brakeOn && !oldDriving) {
        UpdateVoltages();
    }
}

// Calculate radius from x stick, and drive
void Drivetrain::Drive (double yInput, double xInput) {
    // Radius math in Desmos (https://www.desmos.com/calculator/htvtwcp39g)
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
    SetDrivingMode(true);

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
    rightWheelSpeed *= speed;

    // Write to motors
    leftGroup.Set(leftWheelSpeed);
    rightGroup.Set(rightWheelSpeed);
}

void Drivetrain::SetBrake (bool on) {
    if (on == brakeOn) return;

    brakeOn = on;

    auto mode = on ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;

    leftLeader.SetIdleMode(mode);
    leftFollower1.SetIdleMode(mode);
    leftFollower2.SetIdleMode(mode);
    rightLeader.SetIdleMode(mode);
    rightFollower1.SetIdleMode(mode);
    rightFollower2.SetIdleMode(mode);

    leftGroup.Set(0);
    rightGroup.Set(0);
}

void Drivetrain::SetPose (double x, double y, double angle) {
    leftLeader.GetEncoder().SetPosition(0);
    rightLeader.GetEncoder().SetPosition(0);
    odometry.ResetPosition(
        frc::Pose2d{frc::Translation2d{units::length::meter_t{x}, units::length::meter_t{y}}, frc::Rotation2d{units::radian_t{angle}}},
        frc::Rotation2d{units::degree_t{-gyro.GetAngle()}}
    );
}

void Drivetrain::UpdateOdometry () {
    odometry.Update(
        frc::Rotation2d{units::degree_t{-gyro.GetAngle()}},
        units::meter_t{leftLeader.GetEncoder().GetPosition()},
        units::meter_t{rightLeader.GetEncoder().GetPosition()}
    );

    // auto pose = odometry.GetPose();
    // std::cout << ", " << pose.Translation().X().to<double>() << ", " << pose.Translation().Y().to<double>() << ", " << pose.Rotation().Degrees().to<double>();
}

void Drivetrain::UpdateVoltages () {
    double linearVoltage = GetLinearVoltage();
    double rotationalVoltage = GetRotationalVoltage();

    voltageUsedWithoutAcceleration = std::fabs(linearVoltage) + std::fabs(rotationalVoltage) - config.kinematics.ka*acceleration;

    leftGroup.Set((linearVoltage-rotationalVoltage) / leftLeader.GetBusVoltage());
    rightGroup.Set((linearVoltage+rotationalVoltage) / rightLeader.GetBusVoltage());
}

double Drivetrain::GetLinearVoltage () {
    double speed = GetSpeed();
    double a = acceleration + GetAccelerationCorrection(speed);
    double s = std::fabs(speed) > 0.02 ? std::copysign(config.kinematics.ks, speed) : (std::fabs(a) > 0.05 ? std::copysign(config.kinematics.ks, a) : 0);
    double voltage = s + config.kinematics.kv*speed + config.kinematics.ka*a;

    // std::cout << speed << ", " << a << ", ";

    return voltage;
}

double Drivetrain::GetRotationalVoltage () {
    double w = angularVelocity + GetRotationalCorrection();
    double voltage = config.kinematics.kw*w;
    // std::cout << ", " << w/90.0 << ", " << GetGyroAngle()/90.0;
    return voltage;
}

#define kVCorrection 3.5
#define vCorrectionMax 0.7
#define kACorrection 3.0
#define aCorrectionMax 0.7

double Drivetrain::GetAccelerationCorrection (double speed) {
    double correction = kVCorrection * (targetSpeed - speed);
    return std::clamp(correction, -vCorrectionMax, vCorrectionMax);
}

double Drivetrain::GetRotationalCorrection () {
    double angle = odometry.GetPose().Rotation().Radians().to<double>();
    double correction = kACorrection * (targetAngle - angle);
    return std::clamp(correction, -aCorrectionMax, aCorrectionMax);
}
