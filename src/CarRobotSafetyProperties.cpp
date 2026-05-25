#include "CarRobotSafetyProperties.hpp"

CarRobotSafetyProperties::CarRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),

      slSystemOff("System is offline"),
      slSystemOn("System is online"),
      slShuttingDown("System is shutting down"),
      slBraking("Robot is breaking"),
      slStartingUp("System is starting up"),
      slEmergency("Emergency state"),
      slEmergencyBraking("Emergency braking"),
      slMotorPowerOn("Motor power is on"),
      slSystemMoving("Robot is moving"),

      doSystemOn("Startup the system"),
      systemStarted("System has started"),
      powerOn("Power on the system"),
      powerOff("Power off the system"),
      startMoving("Start moving"),
      stopMoving("Stop moving"),
      emergency("Emergency event"),
      resetEmergency("Reset emergency state"),
      shutDown("Shut down the system"),
      abort("Abort current operation"),
      motorsHalted("Motors are halted")
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    // ... = hal.getLogicOutput("...");
    greenLED = hal.getLogicOutput("onBoardLEDgreen");
    redLED = hal.getLogicOutput("onBoardLEDred");

    criticalOutputs = { greenLED, redLED };
    // criticalOutputs = { ... };

    // Declare and add critical inputs
    // ... = eeros::hal::HAL::instance().getLogicInput("...", ...);
    buttonPause = eeros::hal::HAL::instance().getLogicInput("onBoardButtonPause");
    buttonMode = eeros::hal::HAL::instance().getLogicInput("onBoardButtonMode");
    // criticalInputs = { ... };

    criticalInputs = { buttonPause, buttonMode };

    // Add all safety levels to the safety system
    addLevel(slSystemOff);
    addLevel(slSystemOn);
    addLevel(slShuttingDown);
    addLevel(slBraking);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBraking);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(shutDown, slSystemOff, kPublicEvent);
    slBraking.addEvent(motorsHalted, slShuttingDown, kPublicEvent);
    
    slStartingUp.addEvent(systemStarted, slSystemOn, kPublicEvent);
    
    slEmergency.addEvent(resetEmergency, slSystemOn, kPublicEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPublicEvent);
    
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergencyBraking, kPublicEvent);
    slSystemMoving.addEvent(abort, slBraking, kPublicEvent);
    
    addEventToAllLevelsBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelsBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);

    slSystemOff.setInputActions({ ignore(buttonPause), ignore(buttonMode) });
    slShuttingDown.setInputActions({ ignore(buttonPause), ignore(buttonMode) });
    slBraking.setInputActions({ ignore(buttonPause), ignore(buttonMode) });
    slStartingUp.setInputActions({ ignore(buttonPause), ignore(buttonMode) });
    slEmergency.setInputActions({ ignore(buttonPause), check(buttonMode, false, resetEmergency) });
    slEmergencyBraking.setInputActions({ ignore(buttonPause), ignore(buttonMode) });
    slSystemOn.setInputActions({ check(buttonPause, false, emergency), ignore(buttonMode) });
    slMotorPowerOn.setInputActions({ check(buttonPause, false, emergency), ignore(buttonMode) });
    slSystemMoving.setInputActions({ check(buttonPause, false, emergency), ignore(buttonMode) });

    slSystemOff.setOutputActions({set(greenLED, false), set(redLED, false)});
    slShuttingDown.setOutputActions({set(greenLED, false), set(redLED,true)});
    slBraking.setOutputActions({set(greenLED, false), set(redLED,true)});
    slStartingUp.setOutputActions({set(greenLED, true), set(redLED,false)});
    slEmergency.setOutputActions({set(greenLED, true), set(redLED,true)});
    slEmergencyBraking.setOutputActions({set(greenLED, true), set(redLED,true)});
    slSystemOn.setOutputActions({set(greenLED, true), set(redLED,false)});
    slMotorPowerOn.setOutputActions({set(greenLED, true), set(redLED,false)});
    slSystemMoving.setOutputActions({set(greenLED, true), set(redLED,false)});

    // Define and add level actions
    
    slSystemOff.setLevelAction([&] (SafetyContext *privateContext) {
        // Actions to execute when the system is offline
        eeros::Executor::stop();
    });

    slShuttingDown.setLevelAction([&] (SafetyContext *privateContext) {
        // Actions to execute when the system is shutting down
        cs.timedomain.stop();
        privateContext->triggerEvent(shutDown);
    });

    slBraking.setLevelAction([&] (SafetyContext *privateContext) {
        // Actions to execute when the robot is braking
        privateContext->triggerEvent(motorsHalted);
    });

    slStartingUp.setLevelAction([&] (SafetyContext *privateContext) {
        // Actions to execute when the system is starting up
        cs.timedomain.start(); 
        privateContext->triggerEvent(systemStarted);
    });

    slEmergency.setLevelAction([&] (SafetyContext *privateContext) {
        // Actions to execute when the system is in an emergency state
    });

    slEmergencyBraking.setLevelAction([&] (SafetyContext *privateContext) {
        // Actions to execute when the system is in an emergency braking state
        privateContext->triggerEvent(motorsHalted);
    });

    slSystemOn.setLevelAction([&, dt] (SafetyContext *privateContext) {
        // Actions to execute when the system is online
        if(slSystemOn.getNofActivations() * dt >= 1) {
            privateContext->triggerEvent(powerOn);
        }
    });

    slMotorPowerOn.setLevelAction([&, dt] (SafetyContext *privateContext) {
        // Actions to execute when the motor power is on
        if(slMotorPowerOn.getNofActivations() * dt >= 5) {
            privateContext->triggerEvent(startMoving);
        }
    });

    slSystemMoving.setLevelAction([&, dt] (SafetyContext *privateContext) {
        // Actions to execute when the robot is moving
        if(slSystemMoving.getNofActivations() * dt >= 5) {
            privateContext->triggerEvent(stopMoving);
        }
    });


    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(abort);
    });
}
