#include "subsystems/Drivetrain.h"

#include <iostream>
#include <math.h>

#include "Robot.h"

#define PI 3.14159265358979323846

#define kTurnInputConstant  0.2

constexpr units::length::inch_t kHalfWheelBase{(23.0 + 7.0/8.0) / 2.0};
constexpr double kWheelRadiansPerMotorRotation = (1 / 10.71) * (2 * PI); // Encoder ticks per radian

Drivetrain::Drivetrain () {}

void Drivetrain::Periodic () {
    // m_OdometryHelper.Update();
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

    RadiusDrive(yInput, units::length::foot_t(r));
}

// Given a radius and speed, drive around the circle
template<typename LengthUnit>
void Drivetrain::RadiusDrive (double speed, LengthUnit radius) {
    static_assert(units::traits::is_length_unit<LengthUnit>::value, "Input value radius must represent a length quantity.");

    radius *= -1;

    // Calculate the radius for each wheel
    LengthUnit leftWheelRadius = radius + kHalfWheelBase;
    LengthUnit rightWheelRadius = radius - kHalfWheelBase;

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
