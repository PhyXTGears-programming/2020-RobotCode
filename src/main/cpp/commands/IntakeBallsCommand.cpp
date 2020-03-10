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

void IntakeBallsCommand::Initialize () {
    m_Intake->ConveyorStart();
    m_Intake->IntakeStart();

    if (!m_Intake->IsPowerCellInFeeder()) {
        m_Intake->FeedLoadStart();
    }
}

void IntakeBallsCommand::Execute () {
    std::cout << "Balls: " << m_PowerCellCounter->GetCount() << std::endl;
    // if (m_PowerCellCounter->GetCount() >= 5) {
    //     // If robot has 5 balls, stop intake & expel balls in intake
    //     m_Intake->IntakeReverse();
    //     m_Intake->ConveyorStop();
    // }
    if (m_Intake->IsPowerCellInFeeder()) {
        m_Intake->FeedStop();
    }
}

void IntakeBallsCommand::End (bool interrupted) {
    m_Intake->IntakeStop();
    m_Intake->ConveyorStop();
    m_Intake->FeedStop();
}

bool IntakeBallsCommand::IsFinished() {
    // return (m_PowerCellCounter->GetCount() >= 5);
    return false;
}
