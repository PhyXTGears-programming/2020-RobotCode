#include "subsystems/PowerCellCounter.h"

PowerCellCounter::PowerCellCounter () {
    // Trigger interupt on rising edge, not on falling edge
    m_PowerCellIn.SetUpSourceEdge(true, false);
    m_PowerCellOut.SetUpSourceEdge(true, false);
    
    // Update counter when triggered
    m_PowerCellIn.RequestInterrupts([=](frc::InterruptableSensorBase::WaitResult res) { m_Count++; });
    m_PowerCellOut.RequestInterrupts([=](frc::InterruptableSensorBase::WaitResult res) { m_Count--; });

    // activates interupts
    m_PowerCellIn.EnableInterrupts();
    m_PowerCellOut.EnableInterrupts();
}

size_t PowerCellCounter::GetPCCount () {
    return m_Count;
}