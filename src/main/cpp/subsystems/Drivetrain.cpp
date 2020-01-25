#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <iostream>
#include <math.h>

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

#define kHalfWheelBase 0.953125

#define kJoystickDeadzone 0.15

// #define maxForwardAcceleration  0.035
// #define maxForwardDeceleration  0.035
// #define maxBackwardAcceleration 0.035
// #define maxBackwardDeceleration 0.025
// #define backwardsStaticSpeed    0.1
// #define forwardsStaticSpeed     -0.1

#define kTurnInputConstant  0.2

Drivetrain::Drivetrain() {
    // Implementation of subsystem constructor goes here.
}

void Drivetrain::Periodic() {
    // Implementation of subsystem periodic method goes here.
}

void Drivetrain::XboxDrive(frc::XboxController & xboxController, double dt) {
    double speedFactor = defaultSpeed;
    speedFactor += xboxController.GetTriggerAxis(frc::XboxController::kRightHand) * (maxSpeed - defaultSpeed);
    speedFactor -= xboxController.GetTriggerAxis(frc::XboxController::kLeftHand) * (defaultSpeed - minSpeed);

    double xInput = xboxController.GetX(frc::XboxController::kRightHand);
    double yInput = -xboxController.GetY(frc::XboxController::kLeftHand);

    // See desmos
    double scalingConstant = 1/(1 - kJoystickDeadzone);
    xInput = (fabs(xInput) < kJoystickDeadzone) ? 0.0 : scalingConstant * (xInput + (xInput < 0 ? kJoystickDeadzone : -kJoystickDeadzone));
    yInput = (fabs(yInput) < kJoystickDeadzone) ? 0.0 : scalingConstant * (yInput + (yInput < 0 ? kJoystickDeadzone : -kJoystickDeadzone));
    
    Drive(speedFactor * yInput, xInput, dt);
}

void Drivetrain::Drive (double yInput, double xInput, double dt) {
    double r = 1/(kTurnInputConstant * xInput) - xInput/kTurnInputConstant;
    if (xInput == -1) {
        r = -0.0;
    }
    std::cout << r << std::endl;
    RadiusDrive(yInput, r, dt);
}

// Given a controller object, use it to drive
void Drivetrain::RadiusDrive(double speed, double radius, double dt) {
    double leftWheelRadius = radius + kHalfWheelBase;
    double rightWheelRadius = radius - kHalfWheelBase;

    std::cout << leftWheelRadius << " " << rightWheelRadius << std::endl;

    double leftWheelSpeed = 1;
    double rightWheelSpeed = 1;

    if (isinf(radius)) {
        // do nothing
    } else if (radius > 0) {
        rightWheelSpeed = fabs(rightWheelRadius) / fabs(leftWheelRadius);
    } else if (radius < 0) {
        leftWheelSpeed = fabs(leftWheelRadius) / fabs(rightWheelRadius);
    }

    leftWheelSpeed *= (signbit(leftWheelRadius) ? -1 : 1) * (signbit(radius) ? -1 : 1);
    rightWheelSpeed *= (signbit(rightWheelRadius) ? -1 : 1) * (signbit(radius) ? -1 : 1);

    leftWheelSpeed *= speed;
    rightWheelSpeed *= -speed;

    std::cout << leftWheelSpeed << " " << rightWheelSpeed << std::endl;

    m_LeftMotors.Set(leftWheelSpeed);
    m_RightMotors.Set(rightWheelSpeed);
}
