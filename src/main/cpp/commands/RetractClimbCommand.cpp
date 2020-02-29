#include "commands/RetractClimbCommand.h"

RetractClimbCommand::RetractClimbCommand (Climb* Climb) {
    AddRequirements(Climb);
    m_Climb = Climb;
}

void RetractClimbCommand::Initialize () {}

void RetractClimbCommand::Execute () {
    m_Climb->PistonRetract();
    m_Climb->SetWinchSpeed(-WINCH_SPEED * 2);
}

void RetractClimbCommand::End (bool interrupted) {}

bool RetractClimbCommand::IsFinished() {
    return false;
}
