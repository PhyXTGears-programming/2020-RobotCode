#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <iostream>

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

#define kWheelRadius 0.953125

// #define maxForwardAcceleration  0.035
// #define maxForwardDeceleration  0.035
// #define maxBackwardAcceleration 0.035
// #define maxBackwardDeceleration 0.025
// #define backwardsStaticSpeed    0.1
// #define forwardsStaticSpeed     -0.1

#define kTurnInputConstant  0.5

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
    double yInput = xboxController.GetY(frc::XboxController::kLeftHand);
    
    Drive(speedFactor * xInput , speedFactor * yInput, dt);
}

void Drivetrain::Drive (double yInput, double xInput, double dt) {
    double r = 1/(kTurnInputConstant * xInput) - xInput/kTurnInputConstant;
    Drive(yInput, r, dt);
}

// Given a controller object, use it to drive
void Drivetrain::RadiusDrive(double speed, double radius, double dt) {
    double leftWheelSpeed = ComputeWheelSpeed(radius, -kWheelRadius);
    double rightWheelSpeed = ComputeWheelSpeed(radius, kWheelRadius);

    m_LeftMotors.Set(leftWheelSpeed);
    m_RightMotors.Set(rightWheelSpeed);
}

double Drivetrain::ComputeWheelSpeed (double radius, double wheelDistance) {
    double radiusAbs = fabs(radius);
    return ((radius - wheelDistance) / (radiusAbs + 1)) * (radius < 0 ? -1 : 1);
}