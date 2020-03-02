#include "commands/RetractClimbCommand.h"

RetractClimbCommand::RetractClimbCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void RetractClimbCommand::Initialize () {
    m_Climb->PistonRetract();
}

void RetractClimbCommand::Execute () {}

void RetractClimbCommand::End (bool interrupted) {}

bool RetractClimbCommand::IsFinished() {
    return true;
}
