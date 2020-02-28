#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <units/units.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include "Constants.h"
#include "kinematics/OdometryHelper.h"

class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain();
        void Periodic() override;

        void Drive(double yInput, double xInput);
        void RadiusDrive(double speed, units::length::meter_t radius);
        void TankDriveVolts(units::volt_t left, units::volt_t right);

    private:
        rev::CANSparkMax m_LeftMotor1 {kLeftMotor1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_LeftMotor2 {kLeftMotor2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_LeftMotor3 {kLeftMotor3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor1 {kRightMotor1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor2 {kRightMotor2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor3 {kRightMotor3, rev::CANSparkMax::MotorType::kBrushless};

        rev::CANPIDController m_LeftMotor1PID {m_LeftMotor1};
        rev::CANPIDController m_LeftMotor2PID {m_LeftMotor2};
        rev::CANPIDController m_LeftMotor3PID {m_LeftMotor3};
        rev::CANPIDController m_RightMotor1PID {m_RightMotor1};
        rev::CANPIDController m_RightMotor2PID {m_RightMotor2};
        rev::CANPIDController m_RightMotor3PID {m_RightMotor3};

        frc::SpeedControllerGroup m_LeftMotors {m_LeftMotor1, m_LeftMotor2, m_LeftMotor3};
        frc::SpeedControllerGroup m_RightMotors {m_RightMotor1, m_RightMotor2, m_RightMotor3};

        rev::CANEncoder m_LeftEncoder {m_LeftMotor1};
        rev::CANEncoder m_RightEncoder {m_RightMotor1};

        OdometryHelper m_OdometryHelper {new rev::CANEncoder(m_LeftMotor1), new rev::CANEncoder(m_RightMotor1)};
};
