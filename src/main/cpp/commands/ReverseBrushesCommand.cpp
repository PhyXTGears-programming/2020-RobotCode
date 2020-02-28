#include "commands/ReverseBrushesCommand.h"

ReverseBrushesCommand::ReverseBrushesCommand (Intake* intake, Drivetrain* drivetrain) {
    AddRequirements(intake);
    m_Intake = intake;
}

void ReverseBrushesCommand::Initialize () {}

void ReverseBrushesCommand::Execute () {
    m_Intake->ConveyorReverse();
}

void ReverseBrushesCommand::End (bool interrupted) {
    m_Intake->ConveyorStop();
}

bool ReverseBrushesCommand::IsFinished() {
    return false;
}
