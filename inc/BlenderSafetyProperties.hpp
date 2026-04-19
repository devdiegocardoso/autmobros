#ifndef BlenderSAFETYPROPERTIES_HPP_
#define BlenderSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class BlenderSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    BlenderSafetyProperties(ControlSystem &cs, double dt);

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

#endif // BlenderSAFETYPROPERTIES_HPP_
