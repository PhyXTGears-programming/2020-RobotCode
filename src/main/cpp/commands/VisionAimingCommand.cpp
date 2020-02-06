#include "commands/VisionAimingCommand.h"

#include <iostream>
#include <math.h>

VisionAimingCommand::VisionAimingCommand (Drivetrain* drivetrain) {
    std::cout << "Start" << std::endl;

    AddRequirements(drivetrain);

    m_Drivetrain = drivetrain;
    
    m_VisionTable = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
}

void VisionAimingCommand::Initialize () {
    m_TurnPID.SetSetpoint(0.0);
    m_TurnPID.SetTolerance(50.0);
}

void VisionAimingCommand::Execute () {
    int count = (int) m_VisionTable->GetNumber("NumShapes", -1);
    double speed = 0;

    if (count > 0) {
        std::cout << "Count: " << count << std::endl;

        double x = -m_VisionTable->GetNumber("Shape1x", 0);
        std::cout << "x: " << x << std::endl;

        speed = m_TurnPID.Calculate(x);
    } else if (count == 0) {
        std::cout << "No objects detected" << std::endl;
    } else {
        std::cout << "Variable NumShapes does not exist in table Vision" << std::endl;

    }

    m_Drivetrain->RadiusDrive(speed, 0);
}