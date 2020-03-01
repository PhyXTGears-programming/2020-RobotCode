#include "subsystems/PowerCellCounter.h"

const std::chrono::milliseconds debounceDelay(1);

PowerCellCounter::PowerCellCounter () {
    using WaitResult = frc::InterruptableSensorBase::WaitResult;

    m_PowerCellInTimestamp = hal::fpga_clock::now();
    m_PowerCellOutTimestamp = hal::fpga_clock::now();
    
    // Update counter when triggered
    m_PowerCellIn.RequestInterrupts(
        [=](WaitResult res) {
            m_PowerCellInTimestamp = hal::fpga_clock::now();
            m_PowerCellInActive = (WaitResult::kRisingEdge == res);
        });
    m_PowerCellOut.RequestInterrupts(
        [=](WaitResult res) {
            m_PowerCellOutTimestamp = hal::fpga_clock::now();
            m_PowerCellOutActive = (WaitResult::kRisingEdge == res);
        });

    // Trigger interupt on rising edge.
    m_PowerCellIn.SetUpSourceEdge(true, false);
    m_PowerCellOut.SetUpSourceEdge(true, false);

    // activates interupts
    m_PowerCellIn.EnableInterrupts();
    m_PowerCellOut.EnableInterrupts();
}

int PowerCellCounter::GetCount () {
    return m_Count;
}

void PowerCellCounter::Periodic () {
    auto now = hal::fpga_clock::now();

    if (m_PowerCellInActive) {
        auto diff = now - m_PowerCellInTimestamp;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
        if (debounceDelay <= millis) {
            m_Count++;
        }
        
        m_PowerCellInActive = false;
    }

    if (m_PowerCellOutActive) {
        auto diff = now - m_PowerCellOutTimestamp;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
        if (debounceDelay <= millis) {
            m_Count--;
        }

        m_PowerCellOutActive = false;
    }
}