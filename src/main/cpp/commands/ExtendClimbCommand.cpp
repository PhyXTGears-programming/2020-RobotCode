#include "commands/ExtendClimbCommand.h"

ExtendClimbCommand::ExtendClimbCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void ExtendClimbCommand::Initialize () {
}

void ExtendClimbCommand::Execute () {
    m_Climb->PistonExtend();
    m_Climb->SetWinchSpeed(WINCH_SPEED);
}

void ExtendClimbCommand::End (bool interrupted) {}

bool ExtendClimbCommand::IsFinished() {
    return false;
}
