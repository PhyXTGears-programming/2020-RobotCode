#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <iostream>
#include <math.h>

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

#define kHalfWheelBase 0.953125

#define kJoystickDeadzone 0.15

#define kTurnInputConstant  0.2

Drivetrain::Drivetrain () {
    // Implementation of subsystem constructor goes here.
}

void Drivetrain::Periodic () {
    // Implementation of subsystem periodic method goes here.
}

// Given an Xbox controller object, use it to drive
void Drivetrain::XboxDrive (frc::XboxController & xboxController, double dt) {
    double speedFactor = defaultSpeed; // When no triggers are pulled, drive at the default speed

    // Scale between default speed and max speed as the right trigger is pulled (analog)
    speedFactor += xboxController.GetTriggerAxis(frc::XboxController::kRightHand) * (maxSpeed - defaultSpeed);
    // Same as previous line, but between default and min speed, with the left trigger
    speedFactor -= xboxController.GetTriggerAxis(frc::XboxController::kLeftHand) * (defaultSpeed - minSpeed);

    // Get inputs from the controller
    double xInput = xboxController.GetX(frc::XboxController::kRightHand);
    double yInput = -xboxController.GetY(frc::XboxController::kLeftHand);

    // See desmos (https://www.desmos.com/calculator/htvtwcp39g)
    double scalingConstant = 1/(1 - kJoystickDeadzone);
    xInput = (fabs(xInput) < kJoystickDeadzone) ? 0.0 : scalingConstant * (xInput + (xInput < 0 ? kJoystickDeadzone : -kJoystickDeadzone));
    yInput = (fabs(yInput) < kJoystickDeadzone) ? 0.0 : scalingConstant * (yInput + (yInput < 0 ? kJoystickDeadzone : -kJoystickDeadzone));
    
    Drive(speedFactor * yInput, xInput, dt);
}

// Calculate radius from x stick, and drive
void Drivetrain::Drive (double yInput, double xInput, double dt) {
    // Calculate the radius (see desmos, same link)
    double r = 1/(kTurnInputConstant * xInput) - xInput/kTurnInputConstant;

    // If the input is all the way left, tell RadiusDrive that it should turn counterclockwise, instead of clockwise (as it does when the radius is +0.0)
    if (xInput == -1) {
        r = -0.0;
    }
    RadiusDrive(yInput, r, dt);
}

// Given a radius and speed, drive around the circle
void Drivetrain::RadiusDrive(double speed, double radius, double dt) {
    // Calculate the radius for each wheel
    double leftWheelRadius = radius + kHalfWheelBase;
    double rightWheelRadius = radius - kHalfWheelBase;

    // Default speed is 1
    double leftWheelSpeed = 1;
    double rightWheelSpeed = 1;

    // Scale the slower wheel (the inside wheel) to a proportion of the faster wheel speed (left at 1)
    if (isinf(radius)) {
        // Do nothing, when driving straight, both wheels drive at 1
    } else if (radius > 0) {
        rightWheelSpeed = fabs(rightWheelRadius) / fabs(leftWheelRadius);
    } else if (radius < 0) {
        leftWheelSpeed = fabs(leftWheelRadius) / fabs(rightWheelRadius);
    }

    // Correct speed signs (needs more explanation)
    leftWheelSpeed *= std::copysign(1.0, leftWheelRadius) * std::copysign(1.0, radius);
    rightWheelSpeed *= std::copysign(1.0, rightWheelRadius) * std::copysign(1.0, radius);

    // Scale speed based on speed input
    leftWheelSpeed *= speed;
    rightWheelSpeed *= -speed;

    // Write to motors
    m_LeftMotors.Set(leftWheelSpeed);
    m_RightMotors.Set(rightWheelSpeed);
}
