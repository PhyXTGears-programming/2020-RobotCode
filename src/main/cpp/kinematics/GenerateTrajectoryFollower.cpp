#include "kinematics/GenerateTrajectoryFollower.h"

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
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_Drivetrain->TankDriveVolts(0_V, 0_V); }, {})
    );
}

frc::Trajectory GenerateTrajectoryFollower::GetTrajectory (frc::DifferentialDriveKinematics kinematics) {
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

//
