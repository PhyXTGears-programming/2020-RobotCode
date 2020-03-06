#include "commands/ExtendClimbCommand.h"

ExtendClimbCommand::ExtendClimbCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void ExtendClimbCommand::Initialize () {
    m_Climb->PistonExtend();
}

void ExtendClimbCommand::Execute () {}

void ExtendClimbCommand::End (bool interrupted) {}

bool ExtendClimbCommand::IsFinished() {
    return true;
}
