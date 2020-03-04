#include "commands/ControlWinchCommand.h"

ControlWinchCommand::ControlWinchCommand (
        Climb* Climb,
        std::function<double(void)> rsLambda,
        std::function<bool(void)> backLambda
) {
    AddRequirements(Climb);
    m_Climb = Climb;
    m_RightStickCheck = rsLambda;
    m_BackButtonCheck = backLambda;
}

void ControlWinchCommand::Initialize () {}

void ControlWinchCommand::Execute () {
    bool backpressed = m_BackButtonCheck();
    double rightsticky = m_RightStickCheck();

    if (backpressed) {
        if (m_Climb->IsWinchLocked()) {
            m_Climb->WinchUnlock();
        } else {
            m_Climb->WinchLock();
        }
    }

    if (!m_Climb->IsWinchLocked() && std::abs(rightsticky) > 0.25) {
        if (rightsticky > 0) {
            m_Climb->WinchCableIn(rightsticky);
        } else if (rightsticky < 0) {
            m_Climb->WinchCableOut(-rightsticky); // just.. trust me.
        }
    }
}

void ControlWinchCommand::End (bool interrupted) {}

bool ControlWinchCommand::IsFinished() {
    return false;
}
