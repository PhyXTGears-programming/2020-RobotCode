#include "kinematics/GenerateTrajectory.h"

#include <frc/trajectory/TrajectoryGenerator.h>

GenerateTrajectory::GenerateTrajectory () {
    frc::Trajectory trajectory = GetTrajectory();    
}

frc::Trajectory GetTrajectory () {
    frc::TrajectoryConfig trajectoryConfig {RobotPhysicalConstants::maxRobotVelocity, RobotPhysicalConstants::maxRobotAcceleration};

    frc::DifferentialDriveKinematics kinematics {RobotPhysicalConstants::wheelBase};

    frc::DifferentialDriveVoltageConstraint voltageConstraint {frc::SimpleMotorFeedforward<units::meters>(
        RobotPhysicalConstants::kS, RobotPhysicalConstants::kV, RobotPhysicalConstants::kA),
        kinematics, 10_V};

    trajectoryConfig.SetKinematics(kinematics);
    trajectoryConfig.AddConstraint(voltageConstraint); 

    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)}, 
        frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
        trajectoryConfig
    );

    return trajectory;
}
