#pragma once

#include <units/units.h>

struct KinematicsConstants {
    // Feedforward Gains
    static constexpr auto kS = 0.183 * (1_V);
    static constexpr auto kV = 87.5 * (1_V / (1_fps));
    static constexpr auto kA = 13.1 * (1_V / (1_fps_sq));

    // Ramsete Controller
    static constexpr double kRamseteB = 2;
    static constexpr double kRamseteZeta = 0.7;
};
