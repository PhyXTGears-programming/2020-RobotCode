#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>


class Intake : public frc2::SubsystemBase {
 public:
  Intake()
  void IntakeStart(); //turns pickup on
  void IntakeStop(); //turns pickup off
  void IntakeReverse(); //reverses pickup for unjam or deload

  /*
    Will be called periodically whenever the CommandScheduler runs.
  */
  void Periodic() override;
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 ctre::phoenix::motorcontrol::can::TalonSRX m_IntakeMotor(kIntakeMotor);
};
