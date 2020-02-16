#include "kinematics/GenerateTrajectory.h"

GenerateTrajectory::GenerateTrajectory () {
    m_TrajectoryConfig.SetKinematics(m_Kinematics);
    m_TrajectoryConfig.AddConstraint(m_VoltageConstraint);
}
