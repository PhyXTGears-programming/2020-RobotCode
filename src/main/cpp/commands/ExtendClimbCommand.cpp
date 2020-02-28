#include "commands/ExtendClimbCommand.h"

ExtendClimbCommand::ExtendClimbCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void ExtendClimbCommand::Initialize () {
}

void ExtendClimbCommand::Execute () {
    m_Climb->PistonExtend();
}

void ExtendClimbCommand::End (bool interrupted) {}

bool ExtendClimbCommand::IsFinished() {
    return true;
}
