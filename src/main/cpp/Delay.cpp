#include "Delay.h"

Delay::Delay (double duration) {
    m_Duration = duration;
}

void Delay::Start () {
    Reset();
    Start();
}

bool Delay::IsDone () {
    return m_Duration <= Get();
}
