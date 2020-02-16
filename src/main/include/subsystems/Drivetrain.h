#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <units/units.h>

#include "Constants.h"
#include "kinematics/OdometryHelper.h"

class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain();

        void Periodic() override;

        void Drive(double yInput, double xInput);

        template<typename LengthUnit>
        void RadiusDrive(double speed, LengthUnit radius);

    private:
        rev::CANSparkMax m_LeftMotor1 {kLeftMotor1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_LeftMotor2 {kLeftMotor2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_LeftMotor3 {kLeftMotor3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor1 {kRightMotor1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor2 {kRightMotor2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_RightMotor3 {kRightMotor3, rev::CANSparkMax::MotorType::kBrushless};

        frc::SpeedControllerGroup m_LeftMotors {m_LeftMotor1, m_LeftMotor2, m_LeftMotor3};
        frc::SpeedControllerGroup m_RightMotors {m_RightMotor1, m_RightMotor2, m_RightMotor3};

        OdometryHelper m_OdometryHelper {new rev::CANEncoder(m_LeftMotor1), new rev::CANEncoder(m_RightMotor1)};
};
