#include "commands/VisionAimingCommand.h"

#include <iostream>
#include <math.h>
#include <units/units.h>

#define kMaxTurretVelocity 20_rpm

VisionAimingCommand::VisionAimingCommand (Shooter* shooter) {
    AddRequirements(shooter);

    m_Shooter = shooter;
    
    m_VisionTable = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
}

void VisionAimingCommand::Initialize () {
    m_TurretPID.SetSetpoint(0.0);
    m_TurretPID.SetTolerance(0.05);
}

void VisionAimingCommand::Execute () {
    int count = (int) m_VisionTable->GetNumber("NumShapes", -1);
    double speed = 0;

    if (count > 0) {
        std::cout << "Count: " << count << std::endl;

        double x = -m_VisionTable->GetNumber("Shape1x", 0);
        std::cout << "x: " << x << std::endl;

        speed = m_TurretPID.Calculate(x);
    } else if (count == 0) {
        std::cout << "No objects detected" << std::endl;
    } else {
        std::cout << "Variable NumShapes does not exist in table Vision" << std::endl;

    }

    m_Shooter->SetTurretSpeed(kMaxTurretVelocity * speed);
}