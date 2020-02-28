#include "kinematics/GenerateTrajectoryFollower.h"

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "RobotPhysicalConstants.h"
#include "kinematics/KinematicsConstants.h"

#define kPDriveVel 1.0

GenerateTrajectoryFollower::GenerateTrajectoryFollower (OdometryHelper* odometryHelper) {
    m_OdometryHelper = odometryHelper;

    m_Trajectory = GetTrajectory(m_DifferentialDriveKinematics);
}

frc2::SequentialCommandGroup* GenerateTrajectoryFollower::GetDriveCommand (Drivetrain* drivetrain) {
    frc2::RamseteCommand ramseteCommand(
        m_Trajectory, [this]() { return m_OdometryHelper->GetRobotPose(); },
        frc::RamseteController(KinematicsConstants::kRamseteB,
                                KinematicsConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            KinematicsConstants::kS, KinematicsConstants::kV, KinematicsConstants::kA),
        m_DifferentialDriveKinematics,
        [this] { return m_OdometryHelper->GetWheelSpeeds(); },
        frc2::PIDController(kPDriveVel, 0, 0),
        frc2::PIDController(kPDriveVel, 0, 0),
        [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
        {&drivetrain});
}

frc::Trajectory GenerateTrajectoryFollower::GetTrajectory (frc::DifferentialDriveKinematics kinematics) {
    frc::TrajectoryConfig trajectoryConfig {RobotPhysicalConstants::maxRobotVelocity, RobotPhysicalConstants::maxRobotAcceleration};

    frc::DifferentialDriveVoltageConstraint voltageConstraint {frc::SimpleMotorFeedforward<units::meters>(
        KinematicsConstants::kS, KinematicsConstants::kV, KinematicsConstants::kA),
        kinematics, 10_V};

    trajectoryConfig.SetKinematics(kinematics);
    trajectoryConfig.AddConstraint(voltageConstraint); 

    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectoryFollower(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)}, 
        frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
        trajectoryConfig
    );

    return trajectory;
}
