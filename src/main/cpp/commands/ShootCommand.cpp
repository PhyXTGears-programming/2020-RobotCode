#include "commands/ShootCommand.h"

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

units::angular_velocity::revolutions_per_minute_t shooterRPM = 4500_rpm; // 2700 for close shooting, 4500 for regular

ShootCommand::ShootCommand (Shooter* shooter, Intake* intake) {
    AddRequirements(shooter);
    AddRequirements(intake);

    m_Shooter = shooter;
    m_Intake = intake;

    frc::SmartDashboard::PutNumber("Shooter RPM", units::unit_cast<double>(shooterRPM));
}

void ShootCommand::Initialize () {
    feederActivated = false;
}

void ShootCommand::Execute () {
    shooterRPM = units::angular_velocity::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Shooter RPM", units::unit_cast<double>(shooterRPM))};
    m_Shooter->SetShooterMotorSpeed(shooterRPM);

    if (!feederActivated && m_Shooter->GetShooterMotorSpeed() > shooterRPM * 0.95) {
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
