package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.commands.PathPlannerCommands;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.StateHandler.MacroState.Status;

/**
 * A class to handle large-scale robot states (macros) such as launching, intaking, climbing, and idling.
 * Each macro state has an associated status to indicate its current condition.
 * A MacroState can still have its status updated while it is not the current nor desired state.
 */
public class StateHandler extends SubsystemBase implements PathplannerSubsystem{
    
    private static final MacroState[] MACRO_STATES = MacroState.values();

    public enum MacroState {
        IDLE(Status.READY);

        private MacroState(Status status) {
            this.status = status;
        }
        
        private Status status;

        public Status getStatus() {
            return status;
        }
        public void setStatus(Status status) {
            this.status = status;
        }
        
        public enum Status {
            WAITING,
            READY
        }
    }
    public Command setMacroStateStatusCommand(MacroState state, MacroState.Status status) {
        return Commands.runOnce(() -> setStatusOf(state, status)).withName("Set" + state.name() + "StatusTo" + status.name());
    }
    public Command setMacroStatesStatusCommand(MacroState[] states, MacroState.Status status) {
        return Commands.runOnce(() -> {
            for (MacroState state : states) {
                setStatusOf(state, status);
            }
        }).withName("SetMultipleStatesStatusTo" + status.name());
    }

    private LoggedDashboardChooser<MacroState> stateChooser = new LoggedDashboardChooser<>("State Chooser");        

    private static MacroState currentState = MacroState.IDLE;
    private static MacroState requestedState = MacroState.IDLE;

    public StateHandler() {
        // Reset all states to default on construction
        for (MacroState state : MACRO_STATES) {
            state.setStatus(state == MacroState.IDLE ? MacroState.Status.READY : MacroState.Status.WAITING);
        }

        stateChooser.addDefaultOption("IDLE", MacroState.IDLE);
        // Example - stateChooser.addOption("SCORING", MacroState.SCORING);

        stateChooser.onChange((state) -> this.requestState(state));

        addPathPlannerCommands();
    }

    @Override
    public void periodic() {
        // Handle state transition
        if (requestedState != currentState) {
            currentState = requestedState;
        }

        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("StateHandler/Current State", getCurrentState().name());
        Logger.recordOutput("StateHandler/Current State Status", getCurrentState().getStatus().name());
        Logger.recordOutput("StateHandler/Requested State", getRequestedState().name());
        for (MacroState state : MACRO_STATES) {
            Logger.recordOutput("StateHandler/" + state.name() + " Status", state.getStatus().name());
        }
    }

    /**
     * Gets the current state.
     * @return The current MacroState.
     */
    public MacroState getCurrentState() {
        return currentState;
    }
    
    /**
     * Forcefully sets the current state and clears the desired state.
     * @param state The new current MacroState.
     */
    public void setCurrentState(MacroState state) {
        currentState = state;
        clearRequestedState();
    }

    public Command setCurrentStateCommand(MacroState state) {
        return Commands.runOnce(() -> setCurrentState(state)).withName("Force" + state.name());
    }

    /**
     * Gets the desired state.
     * @return The desired MacroState.
     */
    public MacroState getRequestedState() {
        return requestedState;
    }

    /**
     * Sets the desired state.
     * @param state The desired MacroState.
     */
    public void requestState(MacroState state) {
        requestedState = state;
    }

    public Command requestStateCommand(MacroState state) {
        return Commands.runOnce(() -> requestState(state)).withName("Request" + state.name());
    }

    /**
     * Clears the desired MacroState, setting it to whatever the current state is.
     */
    public void clearRequestedState() {
        requestedState = currentState;
    }

    public Command clearRequestedStateCommand() {
        return Commands.runOnce(this::clearRequestedState).withName("ClearRequestedState");
    }

    public MacroState.Status getStatusOf(MacroState state) {
        return state.getStatus();
    }
    
    public void setStatusOf(MacroState state, MacroState.Status status) {
        state.setStatus(status);
    }

    // ==================== Trigger Factory Methods ====================

    /**
     * Creates a Trigger that is true when the current state matches the given state.
     * @param state The MacroState to check against.
     * @return A Trigger that is true when currentState == state.
     */
    public static Trigger whenCurrentState(MacroState state) {
        return new Trigger(() -> currentState == state);
    }

    /**
     * Creates a Trigger that is true when the desired state matches the given state.
     * @param state The MacroState to check against.
     * @return A Trigger that is true when desiredState == state.
     */
    public static Trigger whenDesiredState(MacroState state) {
        return new Trigger(() -> requestedState == state);
    }

    /**
     * Creates a Trigger that is true when a state has a specific status.
     * @param state The MacroState to check.
     * @param status The Status to check for.
     * @return A Trigger that is true when state.getStatus() == status.
     */
    public static Trigger whenStateHasStatus(MacroState state, Status status) {
        return new Trigger(() -> state.getStatus() == status);
    }

    /**
     * Creates a Trigger that is true when the current state matches and has READY status.
     * Useful for triggering actions only when a state is fully prepared.
     * @param state The MacroState to check.
     * @return A Trigger that is true when currentState == state AND status == READY.
     */
    public static Trigger whenCurrentStateReady(MacroState state) {
        return new Trigger(() -> currentState == state && state.getStatus() == Status.READY);
    }

    /**
     * Creates a Trigger that is true when the current state matches and has WAITING status.
     * Useful for triggering spin-up or preparation actions.
     * @param state The MacroState to check.
     * @return A Trigger that is true when currentState == state AND status == WAITING.
     */
    public static Trigger whenCurrentStateWaiting(MacroState state) {
        return new Trigger(() -> currentState == state && state.getStatus() == Status.WAITING);
    }

    @Override
    public void addPathPlannerCommands() {
        // TODO: Rinse and repeat with all states when they get added- just copy the format below and change the state in the method calls

        PathPlannerCommands.addCommand("Request Idle State", this.requestStateCommand(MacroState.IDLE));
        
        PathPlannerCommands.addCommand("Force Idle State", this.setCurrentStateCommand(MacroState.IDLE));

        System.out.println("[StateHandler] Added commands to PathPlanner");
    }
}
