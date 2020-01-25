#include "subsystems/Drivetrain.h"
#include "Robot.h"

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

#define kMotorRevolutionsPerWheelRevolution 1.0
#define kFeetPerWheelRevolution             (5.6 * 3.141593 / 12.0)
#define kConversionFactor                   (kFeetPerWheelRevolution / kMotorRevolutionsPerWheelRevolution)

Drivetrain::Drivetrain() {
    m_LeftEncoder.SetPositionConversionFactor(kConversionFactor);
    m_RightEncoder.SetPositionConversionFactor(kConversionFactor);

    m_LeftEncoder.SetVelocityConversionFactor(kConversionFactor);
    m_RightEncoder.SetVelocityConversionFactor(kConversionFactor);

    m_LeftEncoder.SetPosition(0.0);
    m_RightEncoder.SetPosition(0.0);
}

void Drivetrain::Periodic() {
    // Implementation of subsystem periodic method goes here.
}

// Given a controller object, use it to drive
void Drivetrain::Drive(double dt, frc::XboxController& driver) {
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
                    driveSpeed = lastDriveSpeed - maxBackwardAcceleration * dt * 50;
                }
                break;
            case 0b01: // moving backwards, decelerating
                if (a > maxBackwardDeceleration) {
                    driveSpeed = lastDriveSpeed + maxBackwardDeceleration * dt * 50;
                }
                break;
            case 0b10: // moving forwards, decelerating
                if (a > maxForwardDeceleration) {
                    driveSpeed = lastDriveSpeed - maxForwardDeceleration * dt * 50;
                }
                break;
            case 0b11: // moving forwards, accelerating
                if (a > maxForwardAcceleration) {
                    driveSpeed = lastDriveSpeed + maxForwardAcceleration * dt * 50;
                }
                break;
        }
    }

    // Send to drive subsystem
    m_Drivetrain.ArcadeDrive(driveSpeed, xInput * turnFactor, true);

    lastDriveSpeed = driveSpeed;
}

double Drivetrain::GetLeftDistance () {
    return m_LeftEncoder.GetPosition();
}

double Drivetrain::GetRightDistance () {
    return m_RightEncoder.GetPosition();
}