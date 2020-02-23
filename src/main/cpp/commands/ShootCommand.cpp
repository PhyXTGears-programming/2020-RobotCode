#include "commands/ShootCommand.h"

#include <math.h>

#define kShooterRPM 3200_rpm

ShootCommand::ShootCommand (Shooter* shooter) {
    m_Shooter = shooter;
}

void ShootCommand::Initialize () {
    m_Shooter->SetShooterMotorSpeed(kShooterRPM);
}

void ShootCommand::Execute () {}
