#include "subsystems/Climb.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>

#include <algorithm>

#define MAX_WINCH_SPEED 1.0
#define MIN_WINCH_SPEED 0.0

Climb::Climb () {

}

void Climb::Periodic () {}

void Climb::PistonExtend () {
    m_ClimbExtendSolenoid.Set(true);
    m_ClimbRetractSolenoid.Set(false);
    m_IsPistonExtended = true;
}

void Climb::PistonRetract () {
    m_ClimbExtendSolenoid.Set(false);
    m_ClimbRetractSolenoid.Set(true);
    m_IsPistonExtended = false;
}

void Climb::WinchCableOut(double percentSpeed) {
    percentSpeed = std::clamp(percentSpeed, 0.0, 1.0);
    auto speed = (MAX_WINCH_SPEED - MIN_WINCH_SPEED) * percentSpeed + MIN_WINCH_SPEED;
    SetWinchSpeed(speed);

    SetWinchCableOutFlag();
}

void Climb::WinchCableIn(double percentSpeed) {
    percentSpeed = std::clamp(percentSpeed, 0.0, 1.0);
    auto speed = (MAX_WINCH_SPEED - MIN_WINCH_SPEED) * percentSpeed + MIN_WINCH_SPEED;
    SetWinchSpeed(-speed);
}

void Climb::WinchStop() {
    SetWinchSpeed(0.0);
}

void Climb::SetWinchSpeed (double ClimbWinchSpeed) {
    m_ClimbWinchMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ClimbWinchSpeed);
}

void Climb::SetPistonFlag() {
    m_IsPistonExtended = true;
}

void Climb::ResetPistonFlag() {
    m_IsPistonExtended = false;
}

void Climb::SetWinchCableOutFlag() {
    m_IsWinchCableOut = true;
    m_IsClimbing = true;
}
