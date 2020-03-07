#include "commands/ShootCommand.h"

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

ShootCommand::ShootCommand (Shooter* shooter, Intake* intake, units::angular_velocity::revolutions_per_minute_t speed) {
    AddRequirements(shooter);
    AddRequirements(intake);

    m_Shooter = shooter;
    m_Intake = intake;

    m_Speed = speed;

    frc::SmartDashboard::PutNumber("Shooter RPM", units::unit_cast<double>(m_Speed));
}

void ShootCommand::Initialize () {
    feederActivated = false;
}

void ShootCommand::Execute () {
    m_Speed = units::angular_velocity::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Shooter RPM", units::unit_cast<double>(m_Speed))};
    m_Shooter->SetShooterMotorSpeed(m_Speed);

    if (!feederActivated && m_Shooter->GetShooterMotorSpeed() > m_Speed * 0.95) {
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
