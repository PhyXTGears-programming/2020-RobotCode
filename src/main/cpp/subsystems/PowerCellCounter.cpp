#include "subsystems/PowerCellCounter.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

constexpr std::chrono::milliseconds debounceDelay(100);

PowerCellCounter::PowerCellCounter () {
    using WaitResult = frc::InterruptableSensorBase::WaitResult;

    InitNetworkTables();

    m_PowerCellInTimestamp = hal::fpga_clock::now();
    m_PowerCellOutTimestamp = hal::fpga_clock::now();
    
    // Update counter when triggered
    m_PowerCellIn.RequestInterrupts(
        [=](WaitResult res) {
            if (WaitResult::kRisingEdge == res) {
                m_PowerCellInTimestamp = hal::fpga_clock::now();
                m_PowerCellInActive = true;
            }
        });
    m_PowerCellOut.RequestInterrupts(
        [=](WaitResult res) {
            if (WaitResult::kRisingEdge == res) {
                m_PowerCellOutTimestamp = hal::fpga_clock::now();
                m_PowerCellOutActive = true;
            }
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
            m_PowerCellInActive = false;
        }
    }

    if (m_PowerCellOutActive) {
        auto diff = now - m_PowerCellOutTimestamp;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
        if (debounceDelay <= millis) {
            m_Count--;
            m_PowerCellOutActive = false;
        }
    }

    m_Table->GetEntry("cell count").SetDouble(m_Count);
}

void PowerCellCounter::InitNetworkTables () {
    m_Table = nt::NetworkTableInstance::GetDefault().GetTable("power cell counter");

    auto entryCellCount = m_Table->GetEntry("cell count");

    entryCellCount.SetDouble(m_Count);
    entryCellCount.AddListener(
        [=](auto event) {
            if (event.value->IsDouble()) {
                m_Count = (int)event.value->GetDouble();
            }
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE
    );
}