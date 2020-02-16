#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "RobotPhysicalConstants.h"

class GenerateTrajectory {
    public:
        GenerateTrajectory();

    private:
        frc::DifferentialDriveKinematics m_Kinematics {RobotPhysicalConstants::wheelBase};
        frc::TrajectoryConfig m_TrajectoryConfig {RobotPhysicalConstants::maxRobotVelocity, RobotPhysicalConstants::maxRobotAcceleration};
        frc::DifferentialDriveVoltageConstraint m_VoltageConstraint {frc::SimpleMotorFeedforward<units::meters>(
            RobotPhysicalConstants::kS, RobotPhysicalConstants::kV, RobotPhysicalConstants::kA),
            m_Kinematics, 10_V};
};
