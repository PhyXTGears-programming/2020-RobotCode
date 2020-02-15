#include "commands/ExpelIntakeCommand.h"

ExpelIntakeCommand::ExpelIntakeCommand (Intake* intake) {
    //sets required subsystem
    AddRequirements(intake);
    m_Intake = intake;
}

void ExpelIntakeCommand::Initialize () {
}

void ExpelIntakeCommand::Execute () {
    m_Intake->IntakeReverse();
}

void ExpelIntakeCommand::End () {
    // Stops intake
    m_Intake->IntakeStop();
}

bool ExpelIntakeCommand::IsFinished() {
    return false;
}


