#include "Robot.h"

#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>

void Robot::RobotInit () {}

void Robot::RobotPeriodic () {
    frc2::CommandScheduler::GetInstance().Run();

    m_container.PollInput();

    hal::fpga_clock::time_point now = hal::fpga_clock::now();
    m_DeltaTime = std::chrono::duration_cast<std::chrono::microseconds>(now - m_timePrev).count() / 1000000.0;
    m_timePrev = now;

    // Update rate (for logging)
    static int c = 0;
    if (c >= 50) {
        //std::cout << "Update Rate: " << roundf(1/m_DeltaTime) << " Hz" << std::endl;
        c = 0;
    }
    c++;

    // ProfileShooterPID();
}

void Robot::AutonomousInit () {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic () {}

void Robot::TeleopInit () {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }
}

void Robot::TeleopPeriodic() {
    std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    std::string color = "none";
    if (gameData.length() > 0) {
        switch (gameData[0]) {
            case 'B':
                color = "blue";
                break;
            case 'G':
                color = "green";
                break;
            case 'R':
                color = "red";
                break;
            case 'Y':
                color = "yellow";
                break;
        }
    }
    frc::SmartDashboard::PutString("Control Panel Color", color);
}

void Robot::TestPeriodic () {}

void Robot::DisabledInit () {}

void Robot::DisabledPeriodic () {
    m_container.ReportSelectedAuto();
}

void Robot::ProfileShooterPID () {
    hal::fpga_clock::time_point now = hal::fpga_clock::now();

    static bool needsInit = true;
    static auto startTime = now;
    if (IsAutonomous() && IsEnabled()) {
        if (needsInit) {
            needsInit = false;
            startTime = now;
        }

        auto time = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count() / 1000000.0;
        std::cout
            << time
            << "\t"
            << m_container.m_Shooter->MeasureShooterMotorSpeed1()
            << "\t"
            << m_container.m_Shooter->MeasureShooterMotorSpeed2()
            << "\t"
            << m_container.m_PowerCellCounter->GetCount()
            << std::endl;
    } else {
        needsInit = true;
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
