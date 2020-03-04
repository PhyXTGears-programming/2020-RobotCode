#include "commands/ShootCommand.h"

#include <math.h>

#define kShooterRPM 4500_rpm

ShootCommand::ShootCommand (Shooter* shooter, Intake* intake) {
    AddRequirements(shooter);
    AddRequirements(intake);

    m_Shooter = shooter;
    m_Intake = intake;
}

void ShootCommand::Initialize () {
    m_Shooter->SetShooterMotorSpeed(kShooterRPM);

    feederActivated = false;
}

void ShootCommand::Execute () {
    if (!feederActivated && m_Shooter->GetShooterMotorSpeed() > kShooterRPM * 0.95) {
        m_Intake->SetConveyorSpeed(0.8);
        m_Intake->FeedShooterStart();
        feederActivated = true;
    }
}

void ShootCommand::End (bool interrupted) {
    m_Shooter->SetShooterMotorSpeed(0_rpm);
    m_Intake->ConveyorStop();
    m_Intake->FeedStop();
}
