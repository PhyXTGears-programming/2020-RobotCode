#include "commands/TeleopDriveCommand.h"

TeleopDriveCommand::TeleopDriveCommand(Drivetrain* drivetrain, frc::XboxController* driverController) {
    m_Drivetrain = drivetrain;
    m_Controller = driverController;

    AddRequirements(m_Drivetrain);
}

void TeleopDriveCommand::Initialize () {}

void TeleopDriveCommand::Execute () {
    m_Drivetrain->XboxDrive(*m_Controller);
}