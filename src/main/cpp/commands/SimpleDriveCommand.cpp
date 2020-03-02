#include "commands/SimpleDriveCommand.h"

#include <algorithm>

SimpleDriveCommand::SimpleDriveCommand (double speed, double turn, Drivetrain * drivetrain) {
    m_Speed = std::clamp(speed, -1.0, 1.0);
    m_Turn = std::clamp(turn, -1.0, 1.0);
    m_Drivetrain = drivetrain;

    AddRequirements(m_Drivetrain);
}

void SimpleDriveCommand::Initialize () {
    m_Drivetrain->Drive(m_Speed, m_Turn);
}

void SimpleDriveCommand::Execute () {}

void SimpleDriveCommand::End (bool interrupted) {
    if (interrupted) {
        m_Drivetrain->Drive(0.0, 0.0);
    }
}
