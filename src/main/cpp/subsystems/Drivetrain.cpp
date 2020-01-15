#include "subsystems/Drivetrain.h"

#include <iostream>

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

#define defaultTurn 0.6  // Default turn speed
#define maxTurn     1.0 // Maximum sprint turn speed
#define minTurn     0.3 // Minimum slow turn speed

#define maxForwardAcceleration  0.035
#define maxForwardDeceleration  0.035
#define maxBackwardAcceleration 0.035
#define maxBackwardDeceleration 0.025
#define backwardsStaticSpeed    0.1
#define forwardsStaticSpeed     -0.1

Drivetrain::Drivetrain() {
    // Implementation of subsystem constructor goes here.
}

void Drivetrain::Periodic() {
    // Implementation of subsystem periodic method goes here.
}

// Given a controller object, use it to drive
void Drivetrain::Drive(frc::XboxController& driver) {
    // Get input from joysticks
    double xInput = driver.GetX(frc::XboxController::kRightHand);
    double yInput = -driver.GetY(frc::XboxController::kLeftHand);

    // Calculate drive speed factor from triggers
    double speedFactor = defaultSpeed;
    speedFactor += driver.GetTriggerAxis(frc::XboxController::kRightHand) * (maxSpeed - defaultSpeed);
    speedFactor -= driver.GetTriggerAxis(frc::XboxController::kLeftHand) * (defaultSpeed - minSpeed);

    // Calculate turn speed factor from triggers
    double turnFactor = defaultTurn;
    turnFactor += driver.GetTriggerAxis(frc::XboxController::kRightHand) * (maxTurn - defaultTurn);
    turnFactor -= driver.GetTriggerAxis(frc::XboxController::kLeftHand) * (defaultTurn - minTurn);

    double driveSpeed = yInput * speedFactor;
    double a = driveSpeed - lastDriveSpeed;
    int acceleration = a > 0 ? 0b1 : 0b0;
    int velocity = driveSpeed > 0 ? 0b10 : 0b00;
    a = fabs(a);
    if (lastDriveSpeed > 0 && lastDriveSpeed < forwardsStaticSpeed) {
        driveSpeed = driveSpeed < forwardsStaticSpeed ? driveSpeed : forwardsStaticSpeed;
    } else if (lastDriveSpeed < 0 && lastDriveSpeed > backwardsStaticSpeed) {
        driveSpeed = driveSpeed > backwardsStaticSpeed ? driveSpeed : backwardsStaticSpeed;
    } else {
        switch (velocity | acceleration) {
            case 0b00: // moving backwards, accelerating
                if (a > maxBackwardAcceleration) {
                    driveSpeed = lastDriveSpeed - maxBackwardAcceleration;
                }
                break;
            case 0b01: // moving backwards, decelerating
                if (a > maxBackwardDeceleration) {
                    driveSpeed = lastDriveSpeed + maxBackwardDeceleration;
                }
                break;
            case 0b10: // moving forwards, decelerating
                if (a > maxForwardDeceleration) {
                    driveSpeed = lastDriveSpeed - maxForwardDeceleration;
                }
                break;
            case 0b11: // moving forwards, accelerating
                if (a > maxForwardAcceleration) {
                    driveSpeed = lastDriveSpeed + maxForwardAcceleration;
                }
                break;
        }
    }

    // Send to drive subsystem
    m_Drivetrain.ArcadeDrive(driveSpeed, xInput * turnFactor, true);

    lastDriveSpeed = driveSpeed;
}
