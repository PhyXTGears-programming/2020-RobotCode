#include "commands/IntakeBallsCommand.h"

#include <iostream>

IntakeBallsCommand::IntakeBallsCommand (Intake* intake, PowerCellCounter* cellCounter) {
    //sets required subsystem
    AddRequirements(intake);
    // Do not add power cell counter to subsystem requirements.
    // It's read only and controls no output hardware.

    m_Intake = intake;
    m_PowerCellCounter = cellCounter;
}

void IntakeBallsCommand::Initialize () {}

void IntakeBallsCommand::Execute () {
    std::cout << "Balls: " << m_PowerCellCounter->GetCount() << std::endl;
    if (m_PowerCellCounter->GetCount() >= 5) {
        // If robot has 5 balls, stop intake & expel balls in intake
        m_Intake->IntakeReverse();
        m_Intake->ConveyorStop();
    } else {
        // Starts intake
        m_Intake->IntakeStart();
        m_Intake->ConveyorStart();
    }
}

void IntakeBallsCommand::End (bool interrupted) {
    m_Intake->IntakeStop();
    m_Intake->ConveyorStop();
}

bool IntakeBallsCommand::IsFinished() {
    return (m_PowerCellCounter->GetCount() >= 5);
}
