#include "commands/RetractIntakeCommand.h"

RetractIntakeCommand::RetractIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    m_Intake = intake;
}

void RetractIntakeCommand::Initialize () {
    m_Intake->IntakeRetract();
}

void RetractIntakeCommand::Execute () {
}

void RetractIntakeCommand::End (bool interrupted) {}

bool RetractIntakeCommand::IsFinished() {
    return true;
}
