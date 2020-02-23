#include "commands/ShootCommand.h"

#include <math.h>

#define kShooterRPM 4000_rpm

ShootCommand::ShootCommand (Shooter* shooter) {
    m_Shooter = shooter;
}

void ShootCommand::Initialize () {
    m_Shooter->SetShooterMotorSpeed(kShooterRPM);
    m_Shooter->FeederStart();
}

void ShootCommand::Execute () {}

void ShootCommand::End (bool interrupted) {
    m_Shooter->SetShooterMotorSpeed(0_rpm);
    m_Shooter->FeederStop();
}
