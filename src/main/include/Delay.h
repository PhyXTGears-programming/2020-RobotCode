#include <frc/Timer.h>

class Delay : frc::Timer {
    public:
        Delay(double duration);

        void Start();
        bool IsDone();

    private:
        double m_Duration;
};
