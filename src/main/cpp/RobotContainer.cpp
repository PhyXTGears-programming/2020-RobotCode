#include "RobotContainer.h"

#include <iostream>
#include <cmath>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <units/units.h>

enum class Pov : int {
    kRight = 90,
    kLeft = 270,
    kUp = 180,
    kDown = 0
};

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Shooter);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Intake);

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // frc2::JoystickButton aButton{&m_DriverJoystick, 1};
    // aButton.WhenPressed(m_VisionAimingCommand);
    // aButton.WhenPressed(frc2::PrintCommand("Testing")).WhenReleased(frc2::PrintCommand("Released"));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return &m_AutonomousCommand;
}

void RobotContainer::PollInput () {
    // ##################
    // Driver Controls.
    // ##################
    // TODO: ADD CONTROLS AND TUNE THEM

    // ##################
    // Operator Controls.
    // ##################
    // Shooter controls
    if (m_OperatorJoystick.GetXButtonPressed()) {
        m_Shooter.SetTracking(!m_Shooter.GetTracking());
        std::cout << "tracking?: " << m_Shooter.GetTracking() << std::endl;
    }

    double operatorLeftX = m_OperatorJoystick.GetX(frc::GenericHID::JoystickHand::kLeftHand);
    if (std::abs(operatorLeftX) > 0.1) {
        m_Shooter.SetTracking(false);
        m_Shooter.SetTurretSpeed(static_cast<units::angular_velocity::revolutions_per_minute_t>(operatorLeftX) * 25
        );
    }
    
    if (m_OperatorJoystick.GetAButton()) {
        m_ShootCommand.Schedule();
    } else {
        m_ShootCommand.Cancel();
    }

    // Intake / Carwash controls
    if (m_OperatorJoystick.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) {
        std::cout << "bump!" << std::endl;
        if (m_Intake.IsExtended()) {
            m_RetractIntakeCommand.Schedule();
        } else {
            m_ExtendIntakeCommand.Schedule();
        }
    }

    if (m_OperatorJoystick.GetBButtonPressed()) {
        m_Intake.ConveyorStop();
        m_Intake.IntakeStop();
    } else {
        switch (m_OperatorJoystick.GetPOV())
        {
            case static_cast<int>(Pov::kUp):    m_Intake.IntakeStart();     break; //m_IntakeBallsCommand.Schedule ?
            case static_cast<int>(Pov::kDown):  m_Intake.IntakeReverse();   break; //m_ExpelIntakeCommand.Schedule ?
            case static_cast<int>(Pov::kLeft):  m_Intake.ConveyorReverse(); break;
            case static_cast<int>(Pov::kRight): m_Intake.ConveyorStart();   break;
            default: break;
        }
    }

    // Control panel controls
    // TODO: CHANGE TO REAL CODE
    // if (m_OperatorJoystick.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {
    //     if (m_Controller.IsExtended()) {
    //         m_RetractControllerCommand.Schedule();
    //     } else {
    //         m_ExtendControllerCommand.Schedule();
    //     }
    // }
    // if (m_OperatorJoystick.GetYButtonPressed()) {
    //     m_ControlPanelControl.Schedule();
    // }
}
