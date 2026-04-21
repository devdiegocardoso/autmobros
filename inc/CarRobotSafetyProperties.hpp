#ifndef CarRobotSafetyProperties_HPP_
#define CarRobotSafetyProperties_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class CarRobotSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    CarRobotSafetyProperties(ControlSystem &cs, double dt);

    // Define all possible events
    eeros::safety::SafetyEvent doSystemOff;
    eeros::safety::SafetyEvent doSystemOn;

    // Defina all possible levels
    eeros::safety::SafetyLevel slSystemOff;
    eeros::safety::SafetyLevel slSystemOn;

private:
    // Define all critical outputs
    // eeros::hal::Output<bool>* ...;

    // Define all critical inputs
    // eeros::hal::Input<bool>* ...;

    ControlSystem &cs;
};

#endif // CarRobotSafetyProperties_HPP_
