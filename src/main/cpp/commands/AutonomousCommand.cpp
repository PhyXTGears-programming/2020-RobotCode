#include "commands/AutonomousCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "kinematics/OdometryHelper.h"

AutonomousCommand::AutonomousCommand (Drivetrain* drivetrain, Shooter* shooter) {
    // m_Drivetrain = drivetrain;
    // m_Shooter = shooter;

    // OdometryHelper odometryHelper = m_Drivetrain->GetOdometryHelper();
    // m_GenerateTrajectoryFollower = new GenerateTrajectoryFollower(m_Drivetrain, &odometryHelper);
}

void AutonomousCommand::Initialize () {
    //
}

void AutonomousCommand::Execute () {
    //
}

bool AutonomousCommand::IsFinished () {
    //
}
