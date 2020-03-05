#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

#include <hal/cpp/fpga_clock.h>

#include <networktables/NetworkTable.h>

#include "Constants.h"

class PowerCellCounter : public frc2::SubsystemBase {
    public:
        PowerCellCounter();

        int GetCount();

        void Periodic();

    private:
        frc::DigitalInput m_PowerCellIn {kBeamPowerCellIn};
        frc::DigitalInput m_PowerCellOut {kBeamPowerCellOut};

        // All matches start with 3 power cells.
        int m_Count = 3;

        bool m_PowerCellInActive = false;
        bool m_PowerCellOutActive = false;

        hal::fpga_clock::time_point m_PowerCellInTimestamp;
        hal::fpga_clock::time_point m_PowerCellOutTimestamp;

        std::shared_ptr<NetworkTable> m_Table;

        void InitNetworkTables();
};
