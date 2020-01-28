#include "subsystems/Drivetrain.h"

#include <iostream>
#include <math.h>

#include "Robot.h"

// See desmos for deadzone and radius math (https://www.desmos.com/calculator/htvtwcp39g)

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

#define kHalfWheelBase 0.953125

// Deadzone math in Desmos (link at top of file)
#define kJoystickDeadzone 0.15
#define makeValueFullRange(deadzonedInput) (1/(1 - kJoystickDeadzone) * (deadzonedInput - std::copysign(kJoystickDeadzone, deadzonedInput)))
#define deadzone(input) ((fabs(input) < kJoystickDeadzone) ? 0.0 : makeValueFullRange(input))

#define kTurnInputConstant  0.2

Drivetrain::Drivetrain () {}

void Drivetrain::Periodic () {}

// Given an Xbox controller object, use it to drive
void Drivetrain::XboxDrive (frc::XboxController & xboxController) {
    double speedFactor = defaultSpeed; // When no triggers are pulled, drive at the default speed

    // Scale between default speed and max speed as the right trigger is pulled (analog)
    speedFactor += xboxController.GetTriggerAxis(frc::XboxController::kRightHand) * (maxSpeed - defaultSpeed);
    // Same as previous line, but between default and min speed, with the left trigger
    speedFactor -= xboxController.GetTriggerAxis(frc::XboxController::kLeftHand) * (defaultSpeed - minSpeed);

    // Get inputs from the controller
    double xInput = deadzone(xboxController.GetX(frc::XboxController::kRightHand));
    double yInput = -deadzone(xboxController.GetY(frc::XboxController::kLeftHand));
    
    Drive(speedFactor * yInput, xInput);
}

// Calculate radius from x stick, and drive
void Drivetrain::Drive (double yInput, double xInput) {
    // Radius math in Desmos (link at top of file)
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
