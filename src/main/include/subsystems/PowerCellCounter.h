#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

class PowerCellCounter : public frc2::SubsystemBase {
    public:
        PowerCellCounter();

        size_t GetPCCount();

    private:
        frc::DigitalInput m_PowerCellIn {kPowerCellIn};
        frc::DigitalInput m_PowerCellOut {kPowerCellOut}; 
        size_t m_Count = 0;


};
