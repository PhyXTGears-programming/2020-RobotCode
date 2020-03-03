#pragma once

#include <units/units.h>

#define PI 3.141592653589793238462643383

namespace RobotPhysicalConstants {
    // Drivetrain
    static constexpr units::length::inch_t wheelBase {23.0 + 7.0/8.0};
    static constexpr units::length::inch_t halfWheelBase = wheelBase / 2.0;
    static constexpr double wheelRadiansPerMotorRotation = (1 / 10.71) * (2 * PI);

    static constexpr auto maxRobotVelocity = 1_mps;
    static constexpr auto maxRobotAcceleration = 1_mps_sq;

    // Wheels
    static constexpr auto wheelDiameter = 6_in;
    static constexpr auto wheelRadius = wheelDiameter / 2.0;
    static constexpr auto distancePerWheelRadian = wheelRadius / (1_rad);
}
