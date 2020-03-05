#include "commands/AimShootCommand.h"

#include <math.h>

#define kShooterRPM 6000_rpm

AimShootCommand::AimShootCommand (Shooter* shooter, Intake* intake, PowerCellCounter* counter) {
    AddRequirements(shooter);
    AddRequirements(intake);
    // No requirement for read-only PowerCellCounter.

    m_Shooter = shooter;
    m_Intake = intake;
    m_PowerCellCounter = counter;
}

void AimShootCommand::Initialize () {
    m_ShootSpeed = m_Shooter->GetShooterSpeedForDistance();

    m_Shooter->SetTrackingMode(TrackingMode::CameraTracking);
    m_Shooter->SetShooterMotorSpeed(m_ShootSpeed);

    feederActivated = false;
}

void AimShootCommand::Execute () {
    if (!feederActivated && m_Shooter->GetShooterMotorSpeed() > m_ShootSpeed * 0.95) {
        m_Intake->SetConveyorSpeed(0.8);
        m_Intake->FeedShooterStart();
        feederActivated = true;
    }

    if (feederActivated && m_Shooter->GetShooterMotorSpeed() < m_ShootSpeed * 0.95) {
        m_Intake->FeedStop();
        feederActivated = false;
    }
}

void AimShootCommand::End (bool interrupted) {
    m_Shooter->SetTrackingMode(TrackingMode::Off);
    m_Shooter->SetShooterMotorSpeed(0_rpm);
    m_Intake->ConveyorStop();
    m_Intake->FeedStop();
}

bool AimShootCommand::IsFinished () {
    return 0 == m_PowerCellCounter->GetCount();
}
