#include "commands/AimShootCommand.h"

#include <math.h>

#define kShooterRPM 4500_rpm

AimShootCommand::AimShootCommand (Shooter* shooter, Intake* intake) {
    AddRequirements(shooter);
    AddRequirements(intake);

    m_Shooter = shooter;
    m_Intake = intake;
}

void AimShootCommand::Initialize () {
    m_Shooter->SetTrackingMode(TrackingMode::CameraTracking);
    m_Shooter->SetShooterMotorSpeed(kShooterRPM);

    feederActivated = false;
}

void AimShootCommand::Execute () {
    if (!feederActivated && m_Shooter->GetShooterMotorSpeed() > kShooterRPM * 0.95) {
        m_Intake->SetConveyorSpeed(0.8);
        m_Intake->FeedShooterStart();
        feederActivated = true;
    }
}

void AimShootCommand::End (bool interrupted) {
    m_Shooter->SetTrackingMode(TrackingMode::Off);
    m_Shooter->SetShooterMotorSpeed(0_rpm);
    m_Intake->ConveyorStop();
    m_Intake->FeedStop();
}
