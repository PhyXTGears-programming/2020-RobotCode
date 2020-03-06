#include "commands/RollClimbRightCommand.h"

RollClimbRightCommand::RollClimbRightCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void RollClimbRightCommand::Initialize () {
    m_Climb->RollRight();
}

void RollClimbRightCommand::Execute () {}

void RollClimbRightCommand::End (bool interrupted) {
    m_Climb->RollStop();
}

bool RollClimbRightCommand::IsFinished() {
    return false;
}
