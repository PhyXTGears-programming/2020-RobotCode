#include "commands/ExtendIntakeCommand.h"

ExtendIntakeCommand::ExtendIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    m_Intake = intake;
}

void ExtendIntakeCommand::Initialize () {
}

void ExtendIntakeCommand::Execute () {
    m_Intake->IntakeExtend();
}

void ExtendIntakeCommand::End (bool interrupted) {}

bool ExtendIntakeCommand::IsFinished() {
    return true;
}
