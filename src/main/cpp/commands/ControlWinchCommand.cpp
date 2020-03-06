#include "commands/ControlWinchCommand.h"


ControlWinchCommand::ControlWinchCommand (Climb* Climb, std::function<double(void)> speedLambda) {
    AddRequirements(Climb);
    m_Climb = Climb;
    m_SpeedCheck = speedLambda;
}

void ControlWinchCommand::Initialize () {}

void ControlWinchCommand::Execute () {
    double rightStickY = m_SpeedCheck();

    if (!m_Climb->IsWinchLocked() && std::abs(rightStickY) > 0.25) {
        if (rightStickY > 0) {
            m_Climb->WinchCableIn(rightStickY);
        } else if (rightStickY < 0) {
            m_Climb->WinchCableOut(-rightStickY);
        } else {
            m_Climb->WinchStop();
        }
    }
}

void ControlWinchCommand::End (bool interrupted) {
    m_Climb->WinchStop();
}

bool ControlWinchCommand::IsFinished() {
    return false;
}
