#pragma once

#include <frc/smartdashboard/SendableChooser.h>

template <class T>
class SendableChooser2 : public frc::SendableChooser<T> {
    public:
        std::string GetSelectedName() {
            return this->m_selected;
        }

        std::string GetDefaultName() {
            return this->m_defaultChoice;
        }

        bool HasSelected() {
            return this->m_haveSelected;
        }
};
