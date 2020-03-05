#include "commands/PreheatShooterCommand.h"

#include <math.h>

#define kShooterRPM 4500_rpm

PreheatShooterCommand::PreheatShooterCommand (Shooter* shooter) {
    AddRequirements(shooter);

    m_Shooter = shooter;
}

void PreheatShooterCommand::Initialize () {
    m_Shooter->SetShooterMotorSpeed(kShooterRPM);
}

void PreheatShooterCommand::Execute () {}

void PreheatShooterCommand::End (bool interrupted) {}

bool PreheatShooterCommand::IsFinished () {
    return true;
}