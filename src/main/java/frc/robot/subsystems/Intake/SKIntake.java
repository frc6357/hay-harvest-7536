package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Ports.IntakePorts.kIntakePivotFollower;
import static frc.robot.Ports.IntakePorts.kIntakePivotMotor;
import static frc.robot.Ports.IntakePorts.kIntakeRollerMotor;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Konstants.IntakeConstants.kIntakePivotDeployed;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotMotorD;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotMotorI;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotMotorP;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotStowed;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotTolerance;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerEjectVolts;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerIntakeVolts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class SKIntake extends SubsystemBase {

    SparkFlex intakeRollerMotor = new SparkFlex(kIntakeRollerMotor.ID, MotorType.kBrushless);
    SparkFlex intakePivotMotor = new SparkFlex(kIntakePivotMotor.ID, MotorType.kBrushless);
    SparkFlex intakePivotFollower = new SparkFlex(kIntakePivotFollower.ID, MotorType.kBrushless);

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    SparkFlexConfig pivotFollowerConfig = new SparkFlexConfig();

    @Getter
    @Setter
    public boolean isIntakeDeployed = false;

    public boolean isIntakeAtTarget = false;

    // Last position the pivot was told to go to (or was left at by manual control) - the hold command keeps it here
    @Getter
    private double pivotTargetPosition = kIntakePivotStowed;

    public SKIntake() {

        rollerConfig.inverted(false);
        rollerConfig.idleMode(IdleMode.kCoast); // let the roller spin down instead of jerking pieces
        rollerConfig.smartCurrentLimit(kIntakeRollerCurrentLimit);

        pivotConfig.inverted(false);
        pivotConfig.closedLoop
            .p(kIntakePivotMotorP)
            .i(kIntakePivotMotorI)
            .d(kIntakePivotMotorD)
            .allowedClosedLoopError(kIntakePivotTolerance, ClosedLoopSlot.kSlot0);

        pivotFollowerConfig.follow(intakePivotMotor, true);

        intakeRollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakePivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakePivotFollower.configure(pivotFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Robot must be powered on with the intake stowed, so that's where the encoder starts
        intakePivotMotor.getEncoder().setPosition(kIntakePivotStowed);

        // Whenever no other command is using the intake, hold the pivot at its last target
        setDefaultCommand(holdPivotCommand());
    }

    /** Runs the roller at a voltage - positive pulls game pieces in, negative spits them out */
    public void runIntakeRollers(double volts) {
        intakeRollerMotor.setVoltage(volts);
    }

    public void stopIntakeRollers() {
        intakeRollerMotor.stopMotor();
    }

    /** Runs the roller inward while scheduled, stops it when ended. Keeps holding the pivot meanwhile */
    public Command runRollersIntakeCommand() {
        return rollerCommand(kIntakeRollerIntakeVolts).withName("Intake Rollers In");
    }

    /** Runs the roller outward while scheduled, stops it when ended. Keeps holding the pivot meanwhile */
    public Command runRollersEjectCommand() {
        return rollerCommand(kIntakeRollerEjectVolts).withName("Intake Rollers Eject");
    }

    // Roller commands require this subsystem, which interrupts the hold command, so they keep the pivot held themselves
    private Command rollerCommand(double volts) {
        return runEnd(
            () -> {
                runIntakeRollers(volts);
                setPivotPosition(pivotTargetPosition);
            },
            this::stopIntakeRollers);
    }

    /** Moves the pivot to a position (motor rotations) using the Spark's onboard PID */
    public void setPivotPosition(double position) {
        pivotTargetPosition = position;
        intakePivotMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    /**
     * Drives the pivot open loop (for manual control). The current position becomes the hold target,
     * so the pivot stays wherever it was left. A manual command using this must require this subsystem
     * so it interrupts the hold command instead of fighting it.
     */
    public void runPivotManual(double speed) {
        intakePivotMotor.set(speed);
        pivotTargetPosition = intakePivotMotor.getEncoder().getPosition();
    }

    /** Keeps refreshing the closed loop to the last target position */
    public Command holdPivotCommand() {
        return run(() -> setPivotPosition(pivotTargetPosition)).withName("Hold Intake Pivot");
    }

    public void deployIntake() {
        setPivotPosition(kIntakePivotDeployed);
        isIntakeDeployed = true;
    }

    public void stowIntake() {
        setPivotPosition(kIntakePivotStowed);
        isIntakeDeployed = false;
    }

    /** Stops the pivot where it is and holds it there */
    public void stopIntakePivot() {
        setPivotPosition(intakePivotMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        isIntakeAtTarget = intakePivotMotor.getClosedLoopController().isAtSetpoint();
        logOutputs();
    }

    public void logOutputs() {
        Logger.recordOutput("Intake Roller Motor Speed", intakeRollerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Intake Roller Current", intakeRollerMotor.getOutputCurrent());
        Logger.recordOutput("Intake Pivot Position", intakePivotMotor.getEncoder().getPosition());
        Logger.recordOutput("Intake Pivot Target Position", pivotTargetPosition);
        Logger.recordOutput("Is Intake Pivot At Target Position", isIntakeAtTarget);
    }
    
}
