#include "subsystems/Drivetrain.h"

#include <iostream>
#include <math.h>

#include "Robot.h"
#include "RobotPhysicalConstants.h"

Drivetrain::Drivetrain () {
    // Position in wheel angular displacement (rad)
    m_LeftEncoder.SetPositionConversionFactor(kWheelRadiansPerMotorRotation);
    m_RightEncoder.SetPositionConversionFactor(kWheelRadiansPerMotorRotation);

    // Velocity in wheel angular velocity (rad/s)
    m_LeftEncoder.SetVelocityConversionFactor(kWheelRadiansPerMotorRotation / 60.0);
    m_RightEncoder.SetVelocityConversionFactor(kWheelRadiansPerMotorRotation / 60.0);

    // Initial Position is 0
    m_LeftEncoder.SetPosition(0.0);
    m_RightEncoder.SetPosition(0.0);
}

void Drivetrain::Periodic () {
    m_OdometryHelper.Update();
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

    RadiusDrive(yInput, units::length::foot_t(r));
}

// Given a radius and speed, drive around the circle
void Drivetrain::RadiusDrive (double speed, units::length::meter_t radius) {
    // Reverse turning direction when driving backwards. Special request by Caleb S.
    radius *= std::copysign(1.0, speed);

    // Calculate the radius for each wheel
    units::length::meter_t leftWheelRadius = radius + RobotPhysicalConstants::halfWheelBase;
    units::length::meter_t rightWheelRadius = radius - RobotPhysicalConstants::halfWheelBase;

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
    if (isinf(units::unit_cast<double>(radius))) {
        // Do nothing, when driving straight, both wheels drive at 1
    } else if (radius > units::length::foot_t(0)) { // Right wheel has smaller radius and goes slower
        rightWheelSpeed = units::math::fabs(rightWheelRadius) / units::math::fabs(leftWheelRadius);
    } else if (radius < units::length::foot_t(0)) { // Left wheel has smaller radius and goes slower
        leftWheelSpeed = units::math::fabs(leftWheelRadius) / units::math::fabs(rightWheelRadius);
    }

    // Make one wheel reverse when radius is inside the wheelbase
    leftWheelSpeed *= std::copysign(1.0, units::unit_cast<double>(leftWheelRadius) * std::copysign(1.0, units::unit_cast<double>(radius)));
    rightWheelSpeed *= std::copysign(1.0, units::unit_cast<double>(rightWheelRadius) * std::copysign(1.0, units::unit_cast<double>(radius)));

    // Scale speed based on speed input
    leftWheelSpeed *= -speed; // Negative because right wheels are mounted backwards (one side is always backwards)
    rightWheelSpeed *= speed;

    // Write to motors
    m_LeftMotors.Set(leftWheelSpeed);
    m_RightMotors.Set(rightWheelSpeed);
}

void Drivetrain::TankDriveVolts (units::volt_t left, units::volt_t right) {
    m_LeftMotor1PID.SetReference(units::unit_cast<double>(left), rev::ControlType::kVoltage);
    m_LeftMotor2PID.SetReference(units::unit_cast<double>(left), rev::ControlType::kVoltage);
    m_LeftMotor3PID.SetReference(units::unit_cast<double>(left), rev::ControlType::kVoltage);
    m_RightMotor1PID.SetReference(units::unit_cast<double>(right), rev::ControlType::kVoltage);
    m_RightMotor2PID.SetReference(units::unit_cast<double>(right), rev::ControlType::kVoltage);
    m_RightMotor3PID.SetReference(units::unit_cast<double>(right), rev::ControlType::kVoltage);
}
