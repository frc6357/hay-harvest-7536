package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.Ports.DriverPorts;

public class SK26StateBinder implements CommandBinder {
    private Optional<StateHandler> stateHandlerContainer;
    private StateHandler stateHandler;
    
    Trigger turnOffLaunch;
    Trigger turnOnScoring;
    Trigger turnOnShuttling;
    Trigger turnOnSpitting;

    Trigger launcherReadyToScore;
    Trigger launcherReadyToShuttle;
    Trigger intakeDeployed;
    Trigger intakeAvoidingMajorFouls;
    Trigger turretReadyToScore;
    Trigger turretReadyToShuttle;
    Trigger inAllianceZone;
    Trigger outOfAllianceZone;
    Trigger notNearTower;
    Trigger climbReady;
    Trigger climbStowed;

    public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
        this.stateHandlerContainer = stateHandlerContainer;
        this.stateHandler = stateHandlerContainer.orElse(null);

        // Trigger definitions:
        /* Subsystem States */
        if (stateHandler != null) {

        }

        /* Buttons */
        turnOnScoring = DriverPorts.kRTrigger.button.and(inAllianceZone);
        turnOnShuttling = DriverPorts.kRTrigger.button.and(outOfAllianceZone);
        turnOnSpitting = DriverPorts.kXbutton.button;
    }

    @Override
    public void bindButtons() {
        if(stateHandlerContainer.isEmpty()) {
            return;
        }
        bindRobotStates();
        bindDriverButtons();
        bindOperatorButtons();
    }

    private void bindRobotStates() {
        // Example of binding a state status change to a trigger:
        // launcherReadyToScore.onTrue(stateHandler.setMacroStateStatusCommand(MacroState.LAUNCHING, Status.READY))
        //     .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.LAUNCHING, Status.WAITING));
    }

    private void bindDriverButtons() 
    {
        // Example of binding a button to a state change command:
        // turnOnScoring.onTrue(stateHandler.requestStateCommand(MacroState.SCORING))
        //     .onFalse(stateHandler.requestStateCommand(MacroState.IDLE));
        // kDriver.kXbutton.onTrue(stateHandler.requestStateCommand(MacroState.SPITTING))
        //     .onFalse(stateHandler.requestStateCommand(MacroState.IDLE));
    }

    private void bindOperatorButtons() {
    }
}
