#include "commands/AimCommand.h"

AimCommand::AimCommand (Shooter* shooter) {
    AddRequirements(shooter);

    m_Shooter = shooter;
}

void AimCommand::Initialize () {
    m_Shooter->SetTrackingMode(TrackingMode::CameraTracking);
}

void AimCommand::Execute () {
}

void AimCommand::End (bool interrupted) {
    m_Shooter->SetTrackingMode(TrackingMode::Off);
}

bool AimCommand::IsFinished () {
    return m_Shooter->IsOnTarget();
}