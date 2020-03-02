#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

#include <hal/cpp/fpga_clock.h>


class PowerCellCounter : public frc2::SubsystemBase {
    public:
        PowerCellCounter();

        int GetCount();

        void Periodic();

    private:
        frc::DigitalInput m_PowerCellIn {kBeamPowerCellIn};
        frc::DigitalInput m_PowerCellOut {kBeamPowerCellOut};
        int m_Count = 0;

        bool m_PowerCellInActive = false;
        bool m_PowerCellOutActive = false;

        hal::fpga_clock::time_point m_PowerCellInTimestamp;
        hal::fpga_clock::time_point m_PowerCellOutTimestamp;
};
