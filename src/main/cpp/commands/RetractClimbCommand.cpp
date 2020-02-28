#include "commands/RetractClimbCommand.h"

RetractClimbCommand::RetractClimbCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void RetractClimbCommand::Initialize () {}

void RetractClimbCommand::Execute () {
    m_Climb->PistonRetract();
}

void RetractClimbCommand::End (bool interrupted) {}

bool RetractClimbCommand::IsFinished() {
    return true;
}
