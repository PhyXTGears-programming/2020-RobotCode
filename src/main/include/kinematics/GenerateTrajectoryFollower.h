#pragma once

#include <frc/trajectory/Trajectory.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "OdometryHelper.h"
#include "subsystems/Drivetrain.h"

class GenerateTrajectoryFollower {
    public:
        GenerateTrajectoryFollower(Drivetrain* drivetrain, OdometryHelper* odometryHelper);

        frc2::SequentialCommandGroup* GetDriveCommand();

    private:
        frc::Trajectory GetTrajectory (frc::DifferentialDriveKinematics kinematics);

        Drivetrain* m_Drivetrain;
        OdometryHelper* m_OdometryHelper;

        frc::DifferentialDriveKinematics m_DifferentialDriveKinematics {RobotPhysicalConstants::wheelBase};
        frc::Trajectory m_Trajectory;
};
