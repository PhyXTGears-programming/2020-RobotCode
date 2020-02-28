#include "subsystems/Climb.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>

Climb::Climb () {

}

void Climb::Periodic () {}

void Climb::SetWinchSpeed (double ClimbWinchSpeed) {
    m_ClimbWinchMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ClimbWinchSpeed);
}

void Climb::PistonExtend () {
    m_ClimbExtendSolenoid.Set(true);
    m_ClimbRetractSolenoid.Set(false);
}

void Climb::PistonRetract () {
    m_ClimbExtendSolenoid.Set(false);
    m_ClimbRetractSolenoid.Set(true);
}
