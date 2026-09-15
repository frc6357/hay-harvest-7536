# State Binder Tutorial: Binding Subsystem Triggers to State Status

This tutorial explains the State Binder pattern used in the `bindings` folder, focusing on how subsystem-specific triggers control the status of MacroStates even when those states aren't currently active.

## Overview: The State/Status System

The robot uses a two-layer state management system:

1. **MacroState (Current/Requested)**: What the robot is doing (IDLE, SCORING, INTAKING, etc.)
2. **Status (READY/WAITING)**: Whether a state CAN be executed safely and effectively

**Key Concept**: A state can have its status updated to READY or WAITING **even when it's not the active state**. This allows the robot to know which states are available before switching to them.

### Example Scenario

```
Current State: IDLE
Requested State: IDLE

But in the background:
- SCORING status: WAITING (turret not aimed yet)
- INTAKING status: READY (intake deployed)
- CLIMBING status: WAITING (not in alliance zone)
```

When the driver presses the intake button, the robot can immediately switch to INTAKING because its status is already READY.

## State Binder Architecture

### Structure

```java
public class SK26StateBinder implements CommandBinder {
    private Optional<StateHandler> stateHandlerContainer;
    private StateHandler stateHandler;
    
    // Subsystem state triggers (from other subsystems)
    Trigger launcherReadyToScore;
    Trigger turretReadyToScore;
    Trigger intakeDeployed;
    Trigger inAllianceZone;
    
    // Button triggers (from controllers)
    Trigger turnOnScoring;
    Trigger turnOffLaunch;
    
    @Override
    public void bindButtons() {
        bindRobotStates();  // Bind subsystem triggers to state statuses
        bindDriverButtons(); // Bind controller buttons to state requests
    }
}
```

### Three Types of Bindings

1. **Robot State Bindings**: Subsystem triggers → State status changes
2. **Driver Bindings**: Controller buttons → State requests
3. **Operator Bindings**: Operator buttons → State requests or overrides

## Creating Subsystem-Specific Triggers

### Safe Pattern with Optional Subsystems

When subsystems might not be present, you need to handle `Optional<>` containers safely:

```java
public class SK26StateBinder implements CommandBinder {
    private Optional<StateHandler> stateHandlerContainer;
    private StateHandler stateHandler;
    
    // Declare triggers as fields
    Trigger launcherReadyToScore;
    Trigger turretReadyToScore;
    Trigger intakeDeployed;
    
    public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
        this.stateHandlerContainer = stateHandlerContainer;
        this.stateHandler = stateHandlerContainer.orElse(null);
        
        // Only get triggers if StateHandler exists
        if (stateHandler != null) {
            launcherReadyToScore = stateHandler.getLauncherReadyToScore();
            turretReadyToScore = stateHandler.getTurretReadyToScore();
            intakeDeployed = stateHandler.getIntakeDeployed();
            // ... etc
        }
    }
    
    @Override
    public void bindButtons() {
        // Always check if subsystem exists before binding
        if (stateHandlerContainer.isEmpty()) {
            return;
        }
        bindRobotStates();
        bindDriverButtons();
    }
}
```

### Handling Missing Triggers

If you try to use a trigger that doesn't exist (because StateHandler wasn't created with that subsystem), you'll get errors. Always initialize triggers with safe defaults:

```java
// In StateHandler.java - set safe defaults
@Getter
private Trigger launcherReadyToScore = new Trigger(() -> true);  // Default: always ready
@Getter
private Trigger intakeDeployed = new Trigger(() -> true);        // Default: always ready
@Getter
private Trigger inAllianceZone = new Trigger(() -> false);       // Default: never in zone
```

Then in your binder, check for null before using complex trigger logic:

```java
// Safe approach
if (stateHandler != null && launcherReadyToScore != null) {
    launcherReadyToScore.onTrue(command);
}
```

## Binding Robot States: The Core Pattern

### Basic State Status Binding

The most important bindings connect subsystem readiness to state statuses:

```java
private void bindRobotStates() {
    // SCORING is READY when: in alliance zone AND launcher ready AND turret aimed
    inAllianceZone.and(launcherReadyToScore).and(turretReadyToScore)
        .onTrue(
            stateHandler.setMacroStateStatusCommand(MacroState.SCORING, Status.READY)
        )
        .onFalse(
            stateHandler.setMacroStateStatusCommand(MacroState.SCORING, Status.WAITING)
        );
}
```

**What this does:**
1. Continuously monitors three conditions (alliance zone, launcher, turret)
2. When ALL become true → Sets SCORING status to READY
3. When ANY become false → Sets SCORING status back to WAITING
4. This happens **regardless** of whether SCORING is the current state!

### Multiple States with Same Conditions

Often multiple states share the same readiness requirements:

```java
// SCORING and STEADY_STREAM_SCORING both need the same conditions
inAllianceZone.and(notNearTower).and(launcherReadyToScore).and(turretReadyToScore)
    .onTrue(
        stateHandler.setMacroStateStatusCommand(MacroState.SCORING, Status.READY)
            .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.STEADY_STREAM_SCORING, Status.READY))
    )
    .onFalse(
        stateHandler.setMacroStateStatusCommand(MacroState.SCORING, Status.WAITING)
            .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.STEADY_STREAM_SCORING, Status.WAITING))
    );
```

**Why update multiple states?**
- SCORING = just shooting
- STEADY_STREAM_SCORING = shooting + intaking simultaneously
- Both need the launcher and turret ready, so update both statuses together

### Complex Multi-Subsystem Conditions

Some states require many subsystems to be in specific states:

```java
// CLIMB_AND_SCORE needs: intake stowed, climb ready, in zone, and launcher/turret ready
intakeAvoidingMajorFouls.and(climbReady).and(inAllianceZone)
    .and(launcherReadyToScore).and(turretReadyToScore)
    .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.CLIMB_AND_SCORE, Status.READY))
    .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.CLIMB_AND_SCORE, Status.WAITING));
```

### Field-Based State Availability

Different states are available in different field zones:

```java
// SCORING only available in alliance zone
inAllianceZone.and(launcherReadyToScore).and(turretReadyToScore)
    .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.SCORING, Status.READY));

// SHUTTLING only available outside alliance zone
outOfAllianceZone.and(launcherReadyToShuttle).and(turretReadyToShuttle)
    .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.SHUTTLING, Status.READY));
```

## Binding Driver Buttons: Requesting States

Driver buttons **request** states, but the state only actually activates if its status is READY:

### Simple State Request

```java
private void bindDriverButtons() {
    // Press button → Request INTAKING state
    // Release button → Remove INTAKING from request
    DriverPorts.kLTrigger.button
        .onTrue(stateHandler.addIntakeToRequestedStateCommand())
        .onFalse(stateHandler.removeIntakeFromRequestedStateCommand());
}
```

### Conditional State Requests (Context-Aware)

Some buttons do different things based on field position:

```java
// Single button does two different things based on location
Trigger turnOnScoring = DriverPorts.kRTrigger.button.and(inAllianceZone);
Trigger turnOnShuttling = DriverPorts.kRTrigger.button.and(outOfAllianceZone);

turnOnScoring.onTrue(stateHandler.addScoringToRequestedStateCommand());
turnOnShuttling.onTrue(stateHandler.addShuttlingToRequestedStateCommand());
```

**Result**: Same trigger on controller requests SCORING when in alliance zone, SHUTTLING when outside.

### Multi-Press Detection

Prevent accidental shutoffs with multi-press requirements:

```java
// Create trigger that tracks if launcher has been active
Trigger launcherActive = new Trigger(() -> {
    MacroState state = stateHandler.getCurrentState();
    return state == MacroState.SCORING 
        || state == MacroState.SHUTTLING
        || state == MacroState.STEADY_STREAM_SCORING 
        || state == MacroState.STEADY_STREAM_SHUTTLING;
});

// Require double-press AND 0.8s of active launcher to turn off
Debouncer launcherActiveDebouncer = new Debouncer(0.8, DebounceType.kRising);
Trigger launcherActiveFor800ms = new Trigger(() -> 
    launcherActiveDebouncer.calculate(launcherActive.getAsBoolean())
);

Trigger turnOffLaunch = DriverPorts.kRTrigger.button
    .multiPress(2, 0.33)  // Double-press within 0.33s
    .and(launcherActiveFor800ms);  // AND launcher has been on for 0.8s

turnOffLaunch.onTrue(stateHandler.turnOffLaunchingStatesCommand());
```

## Complete Example: Intake State Binding

Here's a complete example showing how INTAKING state is managed:

```java
public class SK26StateBinder implements CommandBinder {
    private StateHandler stateHandler;
    
    Trigger intakeDeployed;           // From StateHandler (monitors intake subsystem)
    Trigger turnOnIntake;             // From controller
    Trigger turnOffIntake;
    
    public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
        this.stateHandler = stateHandlerContainer.orElse(null);
        
        if (stateHandler != null) {
            // Get subsystem state trigger from StateHandler
            intakeDeployed = stateHandler.getIntakeDeployed();
        }
        
        // Define button triggers
        turnOnIntake = DriverPorts.kLTrigger.button;
        turnOffIntake = DriverPorts.kLTrigger.button.negate();
    }
    
    @Override
    public void bindButtons() {
        if (stateHandler == null) return;
        
        bindRobotStates();
        bindDriverButtons();
    }
    
    private void bindRobotStates() {
        // INTAKING status updates based on intake subsystem
        intakeDeployed
            .onTrue(
                stateHandler.setMacroStateStatusCommand(MacroState.INTAKING, Status.READY)
                    .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.SPITTING, Status.READY))
            )
            .onFalse(
                stateHandler.setMacroStateStatusCommand(MacroState.INTAKING, Status.WAITING)
                    .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.SPITTING, Status.WAITING))
            );
        
        // This binding runs constantly in the background, updating the status
        // Even if current state is IDLE, INTAKING status tracks intake position
    }
    
    private void bindDriverButtons() {
        // Driver requests INTAKING state
        turnOnIntake.onTrue(stateHandler.addIntakeToRequestedStateCommand());
        turnOffIntake.onTrue(stateHandler.removeIntakeFromRequestedStateCommand());
        
        // StateHandler will transition to INTAKING only if:
        // 1. Driver requested it (button pressed)
        // 2. INTAKING status is READY (intake deployed)
    }
}
```

## Common Patterns and Best Practices

### Pattern 1: Exclusive States

Some states are mutually exclusive - you can't do both at once:

```java
// SCORING and SHUTTLING can't both be active
// Use separate triggers based on field position
Trigger turnOnScoring = button.and(inZone);
Trigger turnOnShuttling = button.and(outOfZone);
```

### Pattern 2: Additive States

Some states can be combined:

```java
// INTAKING can be added to IDLE → INTAKING
// INTAKING can be added to SCORING → STEADY_STREAM_SCORING
button.onTrue(stateHandler.addIntakeToRequestedStateCommand());
```

The `addIntakeToRequestedState()` method handles the logic:
```java
public void addIntakeToRequestedState() {
    if(requestedState == MacroState.IDLE) {
        requestedState = MacroState.INTAKING;
    }
    if(requestedState == MacroState.SCORING) {
        requestedState = MacroState.STEADY_STREAM_SCORING;
    }
    // Adds intake to whatever the current requested state is
}
```

### Pattern 3: Safety Interlocks

Prevent dangerous state combinations:

```java
// Can only climb if intake is stowed (avoiding major fouls)
intakeAvoidingMajorFouls.and(climbReady).and(inAllianceZone)
    .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.CLIMBING, Status.READY))
    .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.CLIMBING, Status.WAITING));

// If intake deploys while climbing, CLIMBING becomes WAITING
// This blocks climbing operations until intake is stowed again
```

### Pattern 4: Compound Readiness

States that need multiple subsystems all ready:

```java
// CLIMB_AND_SCORE needs EVERYTHING ready
intakeAvoidingMajorFouls  // Intake stowed
    .and(climbReady)       // Climb at scoring position
    .and(inAllianceZone)   // In correct field zone
    .and(launcherReadyToScore)  // Launcher at speed
    .and(turretReadyToScore)    // Turret aimed
    .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.CLIMB_AND_SCORE, Status.READY));
```

## Troubleshooting: Handling Missing Subsystems

### Problem: NullPointerException when accessing triggers

**Cause**: StateHandler doesn't have that subsystem configured, so the trigger getter returns null.

**Solution**: Always check for null before using triggers:

```java
public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
    this.stateHandler = stateHandlerContainer.orElse(null);
    
    if (stateHandler != null) {
        // Only try to get triggers if StateHandler exists
        launcherReadyToScore = stateHandler.getLauncherReadyToScore();
        turretReadyToScore = stateHandler.getTurretReadyToScore();
    } else {
        // Create dummy triggers if needed
        launcherReadyToScore = new Trigger(() -> false);
        turretReadyToScore = new Trigger(() -> false);
    }
}
```

### Problem: Compilation errors about undefined methods

**Cause**: StateHandler doesn't have getter methods for subsystem triggers because those subsystems weren't set up.

**Solution**: Add triggers to StateHandler with safe defaults:

```java
// In StateHandler.java
@Getter
private Trigger launcherReadyToScore = new Trigger(() -> true);  // Safe default
@Getter  
private Trigger turretReadyToScore = new Trigger(() -> true);    // Safe default

// If subsystem exists, override in constructor or setter:
public void setLauncherSubsystem(Optional<Launcher> launcher) {
    if (launcher.isPresent()) {
        launcherReadyToScore = new Trigger(launcher.get()::atTargetVelocity);
    }
    // If launcher not present, keep the default (always true)
}
```

### Problem: State status never becomes READY

**Cause**: One of the triggers in the compound condition is always false.

**Debug Strategy**:
1. Log each trigger individually to see which is failing:
```java
Logger.recordOutput("Debug/LauncherReady", launcherReadyToScore.getAsBoolean());
Logger.recordOutput("Debug/TurretReady", turretReadyToScore.getAsBoolean());
Logger.recordOutput("Debug/InZone", inAllianceZone.getAsBoolean());
```

2. Temporarily simplify the condition:
```java
// Instead of:
inZone.and(launcherReady).and(turretReady).onTrue(command);

// Try:
inZone.onTrue(command);  // Test just zone
// Then add conditions back one at a time
```

## Example: Creating a New State Binding

Let's walk through adding a new EJECTING state that spits game pieces backward:

### Step 1: Add state to StateHandler

```java
public enum MacroState {
    IDLE(Status.READY),
    SCORING(Status.WAITING),
    INTAKING(Status.WAITING),
    EJECTING(Status.WAITING),  // NEW STATE
    // ... other states
}
```

### Step 2: Determine readiness conditions

EJECTING should be ready when:
- Intake is deployed (to eject through intake)
- Robot is NOT in alliance zone (don't eject in our own zone)
- Have a game piece to eject

### Step 3: Create/identify needed triggers

```java
// In StateHandler.java - add if not present
@Getter
private Trigger intakeDeployed = new Trigger(() -> true);
@Getter  
private Trigger hasGamePiece = new Trigger(() -> true);
@Getter
private Trigger inAllianceZone = new Trigger(() -> false);
```

### Step 4: Add binding in SK26StateBinder

```java
private void bindRobotStates() {
    // ... existing bindings
    
    // EJECTING is ready when intake deployed, has piece, and outside alliance zone
    intakeDeployed.and(hasGamePiece).and(inAllianceZone.negate())
        .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.EJECTING, Status.READY))
        .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.EJECTING, Status.WAITING));
}

private void bindDriverButtons() {
    // Bind to back button (B button)
    DriverPorts.kBbutton.button
        .onTrue(stateHandler.requestStateCommand(MacroState.EJECTING))
        .onFalse(stateHandler.requestStateCommand(MacroState.IDLE));
}
```

### Step 5: Test

1. Check logs to verify status updates correctly
2. Try requesting the state with and without meeting conditions
3. Verify state only activates when status is READY

## Summary: The State Binder Pattern

1. **StateHandler**: Central state machine with current/requested states and statuses
2. **Subsystem Triggers**: Monitor subsystem readiness (from StateHandler getters)
3. **Robot State Bindings**: Connect subsystem triggers to state statuses
4. **Driver Bindings**: Connect controller buttons to state requests
5. **Status Check**: State only activates if status is READY

**Key Insight**: Statuses update continuously in the background based on subsystem conditions, making states "pre-validated" before the driver even requests them. This prevents the robot from entering invalid states and provides smooth, safe operation.

## Additional Resources

- See `TRIGGER_TUTORIAL.md` for more on creating triggers
- See `StateHandler.java` for the state machine implementation
- See `CommandBinder.java` for the binding interface
- See other binders (`SKSwerveBinder.java`, etc.) for more examples
