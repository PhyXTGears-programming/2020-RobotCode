#pragma once

#include <units/units.h>

struct KinematicsConstants {
    // Feedforward Gains
    static constexpr auto kS = 1 * (1_V);
    static constexpr auto kV = 1 * (1_V / (1_mps));
    static constexpr auto kA = 1 * (1_V / (1_mps_sq));

    // Ramsete Controller
    static constexpr double kRamseteB = 2;
    static constexpr double kRamseteZeta = 0.7;
};
