#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "CarRobotSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>
#include "customSteps/SetMotorVoltage.hpp"
#include "customSequences/OrientationException.hpp"
#include <eeros/sequencer/Monitor.hpp>

class MainSequence : public eeros::sequencer::Sequence
{
public:
    MainSequence(std::string name, eeros::sequencer::Sequencer &seq,
                 eeros::safety::SafetySystem &ss,
                 CarRobotSafetyProperties &sp, ControlSystem &cs)
        : eeros::sequencer::Sequence(name, seq),
          ss(ss),
          sp(sp),
          cs(cs),

          sleep("Sleep", this),
          setMotorVoltage("Set Motor Voltage", this, cs),

          checkOrientation(0.1, cs),
          orientationException("Orientation Exception", this, cs, checkOrientation),
        orientationMonitor("Orientation Monitor", this, checkOrientation, eeros::sequencer::SequenceProp::resume, &orientationException)
    {
        addMonitor(&orientationMonitor);
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running)
        {
            setMotorVoltage(-0.5);
            sleep(2.0);
            setMotorVoltage(0.5);
            sleep(2.0);
        }
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    CarRobotSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
    SetMotorVoltage setMotorVoltage;
    CheckOrientation checkOrientation;
    OrientationException orientationException;
    eeros::sequencer::Monitor orientationMonitor;
};

#endif // MAINSEQUENCE_HPP_