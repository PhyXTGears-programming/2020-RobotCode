#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
  void Periodic() override;

  void Drive(double dt, frc::XboxController& driver);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // rev::CANSparkMax m_LeftMotor1 {kLeftDriveMotor1, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax m_LeftMotor2 {kLeftDriveMotor2, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax m_LeftMotor3 {kLeftDriveMotor3, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax m_RightMotor1 {kRightDriveMotor1, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax m_RightMotor2 {kRightDriveMotor2, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax m_RightMotor3 {kRightDriveMotor3, rev::CANSparkMax::MotorType::kBrushless};

  // frc::SpeedControllerGroup m_LeftMotors {m_LeftMotor1, m_LeftMotor2, m_LeftMotor3};
  // frc::SpeedControllerGroup m_RightMotors {m_RightMotor1, m_RightMotor2, m_RightMotor3};

  // frc::DifferentialDrive m_Drivetrain {m_LeftMotors, m_RightMotors};

  double lastDriveSpeed = 0.0;
};
