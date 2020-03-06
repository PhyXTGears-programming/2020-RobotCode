#include "commands/RollClimbLeftCommand.h"

RollClimbLeftCommand::RollClimbLeftCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void RollClimbLeftCommand::Initialize () {
    m_Climb->RollLeft();
}

void RollClimbLeftCommand::Execute () {}

void RollClimbLeftCommand::End (bool interrupted) {
    m_Climb->RollStop();
}

bool RollClimbLeftCommand::IsFinished() {
    return false;
}
