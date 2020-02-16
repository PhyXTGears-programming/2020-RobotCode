#pragma once

#include <units/units.h>

#define PI 3.141592653589793238462643383

namespace RobotPhysicalConstants {
    // Drivetrain
    constexpr units::length::inch_t wheelBase {23.0 + 7.0/8.0};
    constexpr units::length::inch_t halfWheelBase = wheelBase / 2.0;
    constexpr double wheelRadiansPerMotorRotation = (1 / 10.71) * (2 * PI);

    constexpr auto maxRobotVelocity = 1_mps;
    constexpr auto maxRobotAcceleration = 1_mps_sq;

    // Wheels
    constexpr auto wheelDiameter = 5.75_in;
    constexpr auto wheelRadius = wheelDiameter / 2.0;
    constexpr auto distancePerWheelRadian = wheelRadius / (1_rad);

    // Feedforward Gains
    constexpr auto kS = 1 * (1_V);
    constexpr auto kV = 1 * (1_V / (1_mps));
    constexpr auto kA = 1 * (1_V / (1_mps_sq));
}
