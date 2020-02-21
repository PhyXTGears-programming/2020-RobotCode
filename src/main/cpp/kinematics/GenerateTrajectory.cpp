#include "kinematics/GenerateTrajectory.h"

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "RobotPhysicalConstants.h"
#include "kinematics/KinematicsConstants.h"

GenerateTrajectory::GenerateTrajectory (OdometryHelper* odometryHelper) {
    m_OdometryHelper = odometryHelper;

    frc::DifferentialDriveKinematics kinematics {RobotPhysicalConstants::wheelBase};

    frc::Trajectory trajectory = GetTrajectory(kinematics);

    // frc2::RamseteCommand ramseteCommand(
    //     trajectory, [this]() { return m_OdometryHelper->GetRobotPose(); },
    //     frc::RamseteController(KinematicsConstants::kRamseteB,
    //                             KinematicsConstants::kRamseteZeta),
    //     frc::SimpleMotorFeedforward<units::meters>(
    //         KinematicsConstants::kS, KinematicsConstants::kV, KinematicsConstants::kA),
    //     kinematics,
    //     [this] { return m_OdometryHelper->GetWheelSpeeds(); },
    //     frc2::PIDController(DriveConstants::kPDriveVel, 0, 0), // TODO
    //     frc2::PIDController(DriveConstants::kPDriveVel, 0, 0), // TODO
    //     [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); }, // TODO
    //     {&m_drive}); // TODO
}

frc::Trajectory GenerateTrajectory::GetTrajectory (frc::DifferentialDriveKinematics kinematics) {
    frc::TrajectoryConfig trajectoryConfig {RobotPhysicalConstants::maxRobotVelocity, RobotPhysicalConstants::maxRobotAcceleration};

    frc::DifferentialDriveVoltageConstraint voltageConstraint {frc::SimpleMotorFeedforward<units::meters>(
        KinematicsConstants::kS, KinematicsConstants::kV, KinematicsConstants::kA),
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
