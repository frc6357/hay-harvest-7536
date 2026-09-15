# WPILib Trigger Tutorial: Creating and Wiring Triggers

This tutorial explains how to create and wire up Triggers in your FRC robot code, using the StateHandler as an example context.

## What are Triggers?

`Trigger` objects from WPILib's command-based framework allow you to monitor conditions and bind commands to fire when those conditions become true or false. They're a key part of making your robot responsive to subsystem states, sensor inputs, and game conditions.

## Basic Trigger Creation

### 1. Simple Boolean Supplier Trigger

The most basic trigger takes a `BooleanSupplier` (a lambda that returns a boolean):

```java
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Trigger that fires when some condition is true
private Trigger myTrigger = new Trigger(() -> someCondition);

// Example: Trigger when robot X position exceeds 5 meters
private Trigger pastMidfield = new Trigger(() -> swerve.getRobotPose().getX() > 5.0);
```

### 2. Method Reference Trigger

If your subsystem has a method that returns a boolean, you can use method references:

```java
// Assuming your launcher has a method: public boolean atTargetVelocity()
private Trigger launcherReady = new Trigger(launcher::atTargetVelocity);

// This is equivalent to:
private Trigger launcherReady = new Trigger(() -> launcher.atTargetVelocity());
```

### 3. Using Lombok's @Getter for Public Access

If other classes need to access your triggers, use Lombok's `@Getter` annotation:

```java
import lombok.Getter;

@Getter
private Trigger launcherReady = new Trigger(launcher::atTargetVelocity);

// Now other classes can call: stateHandler.getLauncherReady().onTrue(...)
```

## Common Trigger Patterns

### Subsystem State Triggers

Monitor when subsystems reach target positions or states:

```java
// Climb at stowed position
@Getter
private Trigger climbStowed = new Trigger(() -> 
    climb.climbIsAtPosition(ClimbPosition.STOW)
);

// Intake fully deployed
@Getter
private Trigger intakeDeployed = new Trigger(() -> {
    IntakePosition target = intake.getPositionerTargetEnum();
    return intake.getCurrentPosition() <= target.rotations 
        && target == IntakePosition.GROUND;
});
```

### Field Zone Triggers

Create triggers based on robot position relative to field elements:

```java
@Getter
private Trigger inAllianceZone = new Trigger(() -> {
    double robotX = swerve.getRobotPose().getX();
    if (Field.isBlue()) {
        return robotX < LinesVertical.allianceZone + Units.inchesToMeters(chassisLength / 2.0);
    } else {
        return robotX > LinesVertical.redAllianceZone - Units.inchesToMeters(chassisLength / 2.0);
    }
});

// Create the negated trigger
@Getter
private Trigger outOfAllianceZone = inAllianceZone.negate();
```

### Debounced Triggers

Use `.debounce()` to prevent rapid triggering from noisy sensors:

```java
@Getter
private Trigger intakeAvoidingMajorFouls = new Trigger(() -> {
    IntakePosition target = intake.getPositionerTargetEnum();
    return (intake.getCurrentPosition() >= 0.0) 
        && (target == IntakePosition.STOW || target == IntakePosition.FULL_STOW);
}).debounce(0.1, DebounceType.kBoth);  // 100ms debounce on both rising and falling edges
```

## Wiring Triggers with Optional Subsystems

When subsystems may or may not be present (using `Optional<>`), initialize triggers with safe defaults and update them if the subsystem is available:

```java
public class StateHandler extends SubsystemBase {
    
    // Initialize with safe default (always true = ready, or always false = not ready)
    @Getter
    private Trigger launcherReady = new Trigger(() -> true);
    
    /**
     * Call this from RobotContainer after subsystems are constructed
     */
    public void setLauncherSubsystem(Optional<Launcher> launcher) {
        if (launcher.isEmpty()) {
            return;  // Keep the default trigger
        }
        // Replace with actual subsystem state
        launcherReady = new Trigger(launcher.get()::atTargetVelocity);
    }
}
```

In `RobotContainer.java`, wire up the triggers after subsystem creation:

```java
private void configureSubsystems() {
    // Create state handler first
    m_stateHandlerContainer = Optional.of(new StateHandler());
    
    // Create subsystems
    if (subsystems.isLauncherPresent()) {
        m_launcherContainer = Optional.of(new Launcher());
    }
    
    // Wire up triggers
    m_stateHandlerContainer.ifPresent(sh -> 
        sh.setLauncherSubsystem(m_launcherContainer)
    );
}
```

## Binding Commands to Triggers

Once you have triggers, bind commands to them in your button binding classes:

```java
public class MyBinder extends CommandBinder {
    
    @Override
    public void bindButtons() {
        StateHandler state = RobotContainer.m_stateHandlerInstance;
        Launcher launcher = RobotContainer.m_launcherInstance;
        
        // Run command when trigger becomes true
        state.getLauncherReady().onTrue(
            Commands.runOnce(() -> System.out.println("Launcher ready!"))
        );
        
        // Run command while trigger is true
        state.getLauncherReady().whileTrue(
            Commands.run(() -> launcher.shoot())
        );
        
        // Run command when trigger becomes false
        state.getLauncherReady().onFalse(
            Commands.runOnce(() -> launcher.stopMotors())
        );
        
        // Combining triggers
        state.getLauncherReady()
            .and(state.getTurretReady())
            .and(state.getIntakeDeployed())
            .onTrue(Commands.runOnce(() -> System.out.println("All systems go!")));
    }
}
```

## Advanced Trigger Techniques

### Static Trigger Factories

Create reusable trigger factory methods:

```java
/**
 * Creates a Trigger that is true when the current state matches the given state.
 */
public static Trigger whenCurrentState(MacroState state) {
    return new Trigger(() -> currentState == state);
}

// Usage:
whenCurrentState(MacroState.SCORING).onTrue(deployIntakeCommand);
```

### Compound Logic

Combine multiple triggers with boolean operations:

```java
// AND operation
Trigger readyToShoot = launcherReady.and(turretReady).and(hasGamePiece);

// OR operation
Trigger needsAttention = lowBattery.or(motorOverheat).or(sensorDisconnected);

// NOT operation (negate)
Trigger notInSafeZone = inSafeZone.negate();
```

### Trigger Chains

Chain multiple binding methods:

```java
myTrigger
    .onTrue(prepareToShootCommand)
    .whileTrue(aimAtTargetCommand)
    .onFalse(returnToIdleCommand);
```

## Best Practices

1. **Initialize with Safe Defaults**: When using Optional subsystems, initialize triggers with safe default states (e.g., `new Trigger(() -> true)` for "ready" states).

2. **Use Descriptive Names**: Name your triggers clearly to indicate what condition they represent (e.g., `launcherAtSpeed`, `intakeDeployed`, `inDangerZone`).

3. **Debounce Sensor Triggers**: Physical sensors can be noisy; use `.debounce()` to prevent false triggering.

4. **Document Complex Logic**: Add Javadoc comments explaining what conditions the trigger monitors, especially for field-zone or game-specific logic.

5. **Expose with @Getter**: If other parts of your code need to access triggers, make them public using `@Getter` rather than creating manual getter methods.

6. **Test Incrementally**: Test each trigger individually in simulation or on a practice robot before combining them into complex logic.

## Common Pitfalls

- **Null Pointer Exceptions**: Always check if Optional subsystems are present before accessing their methods in trigger lambdas.
- **Rapid Fire**: Without debouncing, triggers can fire multiple times per second on borderline conditions.
- **Resource Intensive Logic**: Avoid heavy computations in trigger lambdas since they're evaluated every 20ms loop cycle.
- **Static vs. Instance**: Be careful with static triggers vs. instance triggers—they have different scopes and lifecycles.

## Example: Complete StateHandler Trigger Setup

Here's a complete example showing how to set up triggers in a StateHandler:

```java
public class StateHandler extends SubsystemBase {
    
    // Initialize with safe defaults
    @Getter
    private Trigger launcherReady = new Trigger(() -> true);
    @Getter
    private Trigger turretReady = new Trigger(() -> true);
    @Getter
    private Trigger intakeDeployed = new Trigger(() -> true);
    @Getter
    private Trigger inAllianceZone = new Trigger(() -> false);
    
    public StateHandler() {
        // Constructor logic
    }
    
    // Wiring methods called from RobotContainer
    public void setLauncherSubsystem(Optional<Launcher> launcher) {
        if (launcher.isEmpty()) return;
        launcherReady = new Trigger(launcher.get()::atTargetVelocity);
    }
    
    public void setTurretSubsystem(Optional<Turret> turret) {
        if (turret.isEmpty()) return;
        turretReady = new Trigger(turret.get()::atTarget);
    }
    
    public void setIntakeSubsystem(Optional<Intake> intake) {
        if (intake.isEmpty()) return;
        intakeDeployed = new Trigger(() -> 
            intake.get().getCurrentPosition() <= IntakePosition.GROUND.rotations
        );
    }
    
    public void setDriveSubsystem(Optional<Swerve> drive) {
        if (drive.isEmpty()) return;
        Swerve swerve = drive.get();
        inAllianceZone = new Trigger(() -> {
            double x = swerve.getRobotPose().getX();
            return Field.isBlue() ? x < ALLIANCE_LINE : x > ALLIANCE_LINE;
        });
    }
}
```

## Resources

- [WPILib Command-Based Documentation](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [WPILib Trigger API](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html)
- [Command-Based Programming Tutorial](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)
