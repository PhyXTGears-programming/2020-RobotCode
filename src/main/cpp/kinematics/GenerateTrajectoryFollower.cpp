#include "kinematics/GenerateTrajectoryFollower.h"

#include <iostream>

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

#include "RobotPhysicalConstants.h"
#include "kinematics/KinematicsConstants.h"

#define kPDriveVel 1.0

GenerateTrajectoryFollower::GenerateTrajectoryFollower (Drivetrain* drivetrain, OdometryHelper* odometryHelper) {
    m_Drivetrain = drivetrain;
    m_OdometryHelper = odometryHelper;
}

frc2::SequentialCommandGroup* GenerateTrajectoryFollower::GetDriveCommand () {
    m_Trajectory = GetTrajectory(m_DifferentialDriveKinematics);

    frc2::RamseteCommand ramseteCommand(
        m_Trajectory,
        [=]() { return m_OdometryHelper->GetRobotPose(); },
        frc::RamseteController(KinematicsConstants::kRamseteB,
                                KinematicsConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            KinematicsConstants::kS, KinematicsConstants::kV, KinematicsConstants::kA),
        m_DifferentialDriveKinematics,
        [=]() { return m_OdometryHelper->GetWheelSpeeds(); },
        frc2::PIDController(kPDriveVel, 0, 0),
        frc2::PIDController(kPDriveVel, 0, 0),
        [=](auto left, auto right) { m_Drivetrain->TankDriveVolts(left, right); },
        {m_Drivetrain}
    );

    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] { std::cout << "-- auto --" << std::endl; }, {}),
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_Drivetrain->TankDriveVolts(0_V, 0_V); }, {m_Drivetrain})
    );
}

frc::Trajectory GenerateTrajectoryFollower::GetTrajectory (frc::DifferentialDriveKinematics kinematics) {
    frc::TrajectoryConfig trajectoryConfig {RobotPhysicalConstants::maxRobotVelocity, RobotPhysicalConstants::maxRobotAcceleration};

    // frc::DifferentialDriveVoltageConstraint voltageConstraint {frc::SimpleMotorFeedforward<units::meters>(
    //     KinematicsConstants::kS, KinematicsConstants::kV, KinematicsConstants::kA),
    //     kinematics, 10_V};

    trajectoryConfig.SetKinematics(kinematics);
    // trajectoryConfig.AddConstraint(voltageConstraint);

    const frc::Pose2d start {0.1_ft, 0.1_ft, frc::Rotation2d(1_deg)};
    const frc::Pose2d end {1_ft, 3_ft, frc::Rotation2d(10_deg)};

    std::vector<frc::Translation2d> waypoints {frc::Translation2d(0.4_ft, 1_ft), frc::Translation2d(0.6_ft, 2_ft)}; // frc::Translation2d(1_ft, 1_ft), frc::Translation2d(2_ft, -1_ft)

    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(start, waypoints, end, trajectoryConfig);

    return trajectory;
}

//
