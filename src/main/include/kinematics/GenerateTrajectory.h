#pragma once

#include <frc/trajectory/Trajectory.h>

#include "OdometryHelper.h"

class GenerateTrajectory {
    public:
        GenerateTrajectory(OdometryHelper* odometryHelper);

    private:
        frc::Trajectory GetTrajectory (frc::DifferentialDriveKinematics kinematics);

        OdometryHelper* m_OdometryHelper;
};
