#include "commands/ExtendIntakeCommand.h"

ExtendIntakeCommand::ExtendIntakeCommand (Intake* intake) {
    //sets required subsystem
    AddRequirements(intake);
    m_Intake = intake;
}

void ExtendIntakeCommand::Initialize () {
}

void ExtendIntakeCommand::Execute () {
    m_Intake->IntakeReverse();
}

void ExtendIntakeCommand::End (bool interrupted) {
    m_Intake->IntakeStop();
}

bool ExtendIntakeCommand::IsFinished() {
    return false;
}
