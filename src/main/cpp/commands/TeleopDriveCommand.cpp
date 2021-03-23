#include "commands/TeleopDriveCommand.h"

#include <math.h>

#define defaultSpeed 0.6 // Default driving speed
#define maxSpeed     1.0 // Maximum sprint speed
#define minSpeed     0.3 // Minimum slow speed

// Deadzone math in Desmos (https://www.desmos.com/calculator/htvtwcp39g)
#define kJoystickDeadzone 0.1
#define makeValueFullRange(deadzonedInput) (1/(1 - kJoystickDeadzone) * (deadzonedInput - std::copysign(kJoystickDeadzone, deadzonedInput)))
#define deadzone(input) ((fabs(input) < kJoystickDeadzone) ? 0.0 : makeValueFullRange(input))

TeleopDriveCommand::TeleopDriveCommand(Drivetrain* drivetrain, frc::XboxController* driverController) {
    m_Drivetrain = drivetrain;
    m_Controller = driverController;

    AddRequirements(m_Drivetrain);
}

void TeleopDriveCommand::Initialize () {}

void TeleopDriveCommand::Execute () {
    double speedFactor = defaultSpeed; // When no triggers are pulled, drive at the default speed

    // Scale between default speed and max speed as the right trigger is pulled (analog)
    speedFactor += m_Controller->GetTriggerAxis(frc::XboxController::kRightHand) * (maxSpeed - defaultSpeed);
    // Same as previous line, but between default and min speed, with the left trigger
    speedFactor -= m_Controller->GetTriggerAxis(frc::XboxController::kLeftHand) * (defaultSpeed - minSpeed);

    // Get inputs from the controller
    double xInput = deadzone(m_Controller->GetX(frc::XboxController::kRightHand));
    double yInput = -deadzone(m_Controller->GetY(frc::XboxController::kLeftHand));
    
    m_Drivetrain->Drive(speedFactor * yInput, xInput);
}