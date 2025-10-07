#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#include <frc/Joystick.h>

class XboxController {
public:
    explicit XboxController(int port) : m_joystick(port) {}

    // Returns the left Y axis value (commonly axis 1)
    double GetLeftY() const {
        return m_joystick.GetRawAxis(1);
    }

    // You can add additional methods here for other axes or buttons.
    
private:
    frc::Joystick m_joystick;
};

#endif  // XBOXCONTROLLER_H
