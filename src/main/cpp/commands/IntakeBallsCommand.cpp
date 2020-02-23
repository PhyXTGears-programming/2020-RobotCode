#include "commands/IntakeBallsCommand.h"

#include <iostream>

IntakeBallsCommand::IntakeBallsCommand (Intake* intake) {
    //sets required subsystem
    AddRequirements(intake);
    m_Intake = intake;
}

void IntakeBallsCommand::Initialize () {}

void IntakeBallsCommand::Execute () {
    std::cout << "Balls: " << m_Intake->GetNumBalls() << std::endl;
    if (m_Intake->GetNumBalls() >= 5) {
        // If robot has 5 balls, stop intake & expel balls in intake
        m_Intake->IntakeReverse();
    } else {
        // Starts intake
        m_Intake->IntakeStart();
    }
}

void IntakeBallsCommand::End (bool interrupted) {
    m_Intake->IntakeStop();
}

bool IntakeBallsCommand::IsFinished() {
    return (m_Intake->GetNumBalls() >= 5);
}
