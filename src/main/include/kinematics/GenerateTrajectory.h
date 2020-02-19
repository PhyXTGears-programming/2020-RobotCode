#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/Trajectory.h>

#include "RobotPhysicalConstants.h"

class GenerateTrajectory {
    public:
        GenerateTrajectory();

    private:
        frc::Trajectory GetTrajectory ();
};
