#include "commands/RetractIntakeCommand.h"

RetractIntakeCommand::RetractIntakeCommand (Intake* intake) {
    //sets required subsystem
    AddRequirements(intake);
    m_Intake = intake;
}

void RetractIntakeCommand::Initialize () {
}

void RetractIntakeCommand::Execute () {
    m_Intake->RetractIntake();
}

void RetractIntakeCommand::End () {
    
}

bool RetractIntakeCommand::IsFinished() {
    return true;
}


