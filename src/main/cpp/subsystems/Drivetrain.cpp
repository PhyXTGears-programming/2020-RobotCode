#include "subsystems/Drivetrain.h"

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

#define defaultTurn 0.6 // Default turn speed
#define maxTurn     1.0 // Maximum sprint turn speed
#define minTurn     0.3 // Minimum slow turn speed

Drivetrain::Drivetrain() {
  // Implementation of subsystem constructor goes here.
}

void Drivetrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  
}

// Given a controller object, use it to drive
void Drivetrain::Drive(frc::XboxController& driver) {
    // Get input from joysticks
    double xInput = -driver.GetX(frc::XboxController::kRightHand);
    double yInput = driver.GetY(frc::XboxController::kLeftHand);

    // Calculate drive speed factor from triggers
    double speedFactor = defaultSpeed;
    speedFactor += driver.GetTriggerAxis(frc::XboxController::kRightHand) * (maxSpeed - defaultSpeed);
    speedFactor -= driver.GetTriggerAxis(frc::XboxController::kRightHand) * (defaultSpeed - minSpeed);

    // Calculate turn speed factor from triggers
    double turnFactor = defaultTurn;
    speedFactor += driver.GetTriggerAxis(frc::XboxController::kRightHand) * (maxTurn - defaultTurn);
    speedFactor -= driver.GetTriggerAxis(frc::XboxController::kRightHand) * (defaultTurn - minTurn);

    // Send to drive subsystem
    m_Drivetrain.ArcadeDrive(yInput * speedFactor, xInput * turnFactor, true);
}
