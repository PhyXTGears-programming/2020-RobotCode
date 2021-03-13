#pragma once

#include <units/units.h>

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain();

        void Periodic() override;

        void Drive(double yInput, double xInput);
        void RadiusDrive(double speed, double radius);

        void SetBrake(bool on);

        units::current::ampere_t GetMotorCurrent () {
            return units::current::ampere_t{(m_LeftMotor1.GetOutputCurrent() + m_RightMotor1.GetOutputCurrent()) / 2.0};
        }

    private:
        rev::CANSparkMax m_LeftMotor1 {DriveMotorPins::Left1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_LeftMotor2 {DriveMotorPins::Left2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_LeftMotor3 {DriveMotorPins::Left3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor1 {DriveMotorPins::Right1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor2 {DriveMotorPins::Right2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor3 {DriveMotorPins::Right3, rev::CANSparkMax::MotorType::kBrushless};

        frc::SpeedControllerGroup m_LeftMotors {m_LeftMotor1, m_LeftMotor2, m_LeftMotor3};
        frc::SpeedControllerGroup m_RightMotors {m_RightMotor1, m_RightMotor2, m_RightMotor3};

        frc::DifferentialDriveOdometry* m_Odometry;
};
