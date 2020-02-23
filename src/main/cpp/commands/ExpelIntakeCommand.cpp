#include "commands/ExpelIntakeCommand.h"

ExpelIntakeCommand::ExpelIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    m_Intake = intake;
}

void ExpelIntakeCommand::Initialize () {}

void ExpelIntakeCommand::Execute () {
    m_Intake->IntakeReverse();
}

void ExpelIntakeCommand::End (bool interrupted) {
    m_Intake->IntakeStop();
}

bool ExpelIntakeCommand::IsFinished() {
    return false;
}
