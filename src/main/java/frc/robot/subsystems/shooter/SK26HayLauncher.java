package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import lombok.Getter;

/**
 * Two-wheel launcher. NEO Vortex motors on SPARK Flex controllers.
 *
 * <p>Setpoints are surface speeds, not RPM. With a 2:1 diameter ratio.
 * 
 * <p>{@code HayLauncher/SpinRatio} is top surface speed over bottom surface speed.
 * 1.0 means matched surfaces and no induced spin, below 1.0 is backspin, above is
 * topspin. Exit speed is roughly the mean of the two surface speeds; spin scales with
 * their difference.
 *
 * <p>Each motor keeps its own closed-loop gains. The 4" wheel carries far more
 * rotational inertia than the two 2" wheels combined, so they do not tune alike even
 * when their surface speeds match.
 *
 * <p><b>Units are motor RPM</b> throughout the control path, because that is what the
 * SPARK's encoder reports natively. Only the public surface-speed API is in m/s.
 *
 * <p>Constants and CAN IDs are inlined below
 */
public class SK26HayLauncher extends SubsystemBase {

    // ==================== CAN IDs ====================
    private static final int kBottomMotorID = 40;
    private static final int kTopMotorID    = 41;

    // ==================== Geometry ====================
    private static final double kBottomWheelDiameterMeters = Inches.of(4.0).in(Meters);
    private static final double kTopWheelDiameterMeters    = Inches.of(2.0).in(Meters);

    /**
     * Motor rotations per wheel rotation. 1.0 is simple same rotato.
     */
    private static final double kBottomGearRatio = 1.0;
    private static final double kTopGearRatio    = 1.0;

    /** Metres of rubber past a fixed point per motor rotation. */
    private static final double kBottomMetersPerMotorRotation =
        Math.PI * kBottomWheelDiameterMeters / kBottomGearRatio;
    private static final double kTopMetersPerMotorRotation =
        Math.PI * kTopWheelDiameterMeters / kTopGearRatio;

    // ==================== Motor / Tuning ====================

    /** NEO Vortex free speed, RPM. Used to seed the velocity feedforward. */
    private static final double kVortexFreeSpeedRPM = 6784.0;

    private static final double kNominalVoltage = 12.0;
    private static final int kSmartCurrentLimit = 40;

    /**
     * Starting kV: volts per RPM. REVLib 2026's feedforward is volts-based, so
     * nominal voltage over free speed is "just enough volts to reach this speed
     * unloaded" — the conventional starting estimate.
     */
    private static final double kStartingKV = kNominalVoltage / kVortexFreeSpeedRPM;

    private static final double kDefaultToleranceRPM = 180.0;
    private static final double kDefaultSurfaceSpeedMPS = 12.75;
    

    private static final class BottomRoller {
        static final double kS = 0.0;
        static final double kV = kStartingKV;
        static final double kP = 0.0001;
        static final double kI = 0.0;
        static final double kD = 0.0;
    }

    private static final class TopRoller {
        static final double kS = 0.0;
        static final double kV = kStartingKV;
        static final double kP = 0.0001;
        static final double kI = 0.0;
        static final double kD = 0.0;
    }

    // Hardware
    private final SparkFlex bottomMotor;
    private final SparkFlex topMotor;
    private final RelativeEncoder bottomEncoder;
    private final RelativeEncoder topEncoder;
    private final SparkClosedLoopController bottomController;
    private final SparkClosedLoopController topController;

    // State
    private double bottomTargetRPM = 0.0;
    private double topTargetRPM = 0.0;
    private double cachedBottomVelocityRPM = 0.0;
    private double cachedTopVelocityRPM = 0.0;

    @Getter
    private boolean running = false;

    /** Top surface speed over bottom surface speed. 1.0 = no spin. */
    private final Pref<Double> spinRatio =
        SKPreferences.attach("HayLauncher/SpinRatio", 1.0);

    /** Manual/dashboard setpoint used by {@link #runVelocityFromPrefCommand()}. */
    private final Pref<Double> manualSurfaceSpeedMPS =
        SKPreferences.attach("HayLauncher/ManualSurfaceSpeed (mps)", kDefaultSurfaceSpeedMPS);

    /** How close each motor must be to its own target to count as "at speed". */
    private final Pref<Double> velocityToleranceRPM =
        SKPreferences.attach("HayLauncher/Tolerance (rpm)", kDefaultToleranceRPM);

    public SK26HayLauncher() {
        bottomMotor = new SparkFlex(kBottomMotorID, MotorType.kBrushless);
        topMotor    = new SparkFlex(kTopMotorID, MotorType.kBrushless);

        bottomEncoder = bottomMotor.getEncoder();
        topEncoder    = topMotor.getEncoder();

        bottomController = bottomMotor.getClosedLoopController();
        topController    = topMotor.getClosedLoopController();

        configureMotors();
    }

    private void configureMotors() {
        SparkFlexConfig bottomConfig = baseConfig();
        bottomConfig.closedLoop.pid(BottomRoller.kP, BottomRoller.kI, BottomRoller.kD);
        bottomConfig.closedLoop.feedForward.sv(BottomRoller.kS, BottomRoller.kV);

        SparkFlexConfig topConfig = baseConfig();
        topConfig.closedLoop.pid(TopRoller.kP, TopRoller.kI, TopRoller.kD);
        topConfig.closedLoop.feedForward.sv(TopRoller.kS, TopRoller.kV);

        var bottomStatus = bottomMotor.configure(
            bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        var topStatus = topMotor.configure(
            topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (bottomStatus != REVLibError.kOk) {
            DriverStation.reportError(
                "[HayLauncher] Bottom motor config failed: " + bottomStatus, false);
        }
        if (topStatus != REVLibError.kOk) {
            DriverStation.reportError(
                "[HayLauncher] Top motor config failed: " + topStatus, false);
        }
    }

    private SparkFlexConfig baseConfig() {
        SparkFlexConfig config = new SparkFlexConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(kNominalVoltage)
            .smartCurrentLimit(kSmartCurrentLimit);

        // Shortest practical velocity filter
        config.encoder
            .uvwMeasurementPeriod(16)
            .uvwAverageDepth(2);

        return config;
    }

    @Override
    public void periodic() {
        // Cache encoder reads (one per motor per cycle)
        cachedBottomVelocityRPM = bottomEncoder.getVelocity();
        cachedTopVelocityRPM    = topEncoder.getVelocity();

        if (running) {
            bottomController.setSetpoint(bottomTargetRPM, ControlType.kVelocity);
            topController.setSetpoint(topTargetRPM, ControlType.kVelocity);
        } else {
            bottomMotor.stopMotor();
            topMotor.stopMotor();
        }

        telemeterize();
    }

    // ==================== Public API ====================

    /**
     * Spin up to a bottom-wheel surface speed. The top wheels are driven to
     * spinRatio times this surface speed, which at a 1.0 ratio is twice the
     * bottom motor's RPM.
     *
     * @param metersPerSecond Target surface speed of the 4" bottom wheel.
     */
    public void runSurfaceSpeed(double metersPerSecond) {
        applySetpoints(metersPerSecond * 60.0 / kBottomMetersPerMotorRotation);
    }

    /**
     * Spin up with the bottom motor at a given RPM. The value is the bottom motor's
     * speed; the top is derived from geometry, not set equal.
     *
     * @param rpm Target bottom motor velocity in rotations per minute.
     */
    public void runVelocityRPM(double rpm) {
        applySetpoints(rpm);
    }

    private void applySetpoints(double bottomRPM) {
        bottomTargetRPM = bottomRPM;

        // Match surface speeds, then scale the top by the spin ratio.
        double bottomSurface = bottomRPM * kBottomMetersPerMotorRotation;
        topTargetRPM = (bottomSurface * spinRatio.get()) / kTopMetersPerMotorRotation;

        running = true;
    }

    /** Stop both rollers and coast to a halt. */
    public void stop() {
        bottomTargetRPM = 0.0;
        topTargetRPM = 0.0;
        running = false;
    }

    /** @return true if both motors are within tolerance of their own target. */
    public boolean atTargetVelocity() {
        return withinTolerance(velocityToleranceRPM.get());
    }

    /**
     * Looser version of {@link #atTargetVelocity()} at 2x tolerance. Use it to start
     * feeding slightly before the wheels have fully settled.
     */
    public boolean closeToTargetVelocity() {
        return withinTolerance(velocityToleranceRPM.get() * 2.0);
    }

    private boolean withinTolerance(double tolerance) {
        if (!running) return false;
        return Math.abs(cachedBottomVelocityRPM - bottomTargetRPM) <= tolerance
            && Math.abs(cachedTopVelocityRPM - topTargetRPM)       <= tolerance;
    }

    /** @return Measured surface speed of the 4" bottom wheel, m/s. */
    public double getBottomSurfaceSpeedMPS() {
        return cachedBottomVelocityRPM / 60.0 * kBottomMetersPerMotorRotation;
    }

    /** @return Measured surface speed of the 2" top wheels, m/s. */
    public double getTopSurfaceSpeedMPS() {
        return cachedTopVelocityRPM / 60.0 * kTopMetersPerMotorRotation;
    }

    /**
     * @return Rough predicted exit speed of the game piece in m/s, the mean of the two
     *         measured surface speeds. Assumes both wheels grip without slipping.
     */
    public double getEstimatedExitSpeedMPS() {
        return (getBottomSurfaceSpeedMPS() + getTopSurfaceSpeedMPS()) / 2.0;
    }

    /** @return Current bottom roller velocity in RPM. */
    public double getBottomVelocityRPM() {
        return cachedBottomVelocityRPM;
    }

    /** @return Current top roller velocity in RPM. */
    public double getTopVelocityRPM() {
        return cachedTopVelocityRPM;
    }

    /** @return Commanded bottom motor velocity in RPM. */
    public double getBottomTargetRPM() {
        return bottomTargetRPM;
    }

    /** @return Commanded top motor velocity in RPM. */
    public double getTopTargetRPM() {
        return topTargetRPM;
    }

    // ==================== Commands ====================

    /**
     * Runs the launcher at the supplied bottom-motor velocity while held, stopping on
     * release. The supplier is read every cycle, so a distance-based lookup tracks as
     * the robot moves, and so does a live edit to the spin ratio.
     *
     * @param velocity Supplier of the target bottom motor angular velocity.
     */
    public Command runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(
            () -> runVelocityRPM(velocity.get().in(RPM)),
            this::stop);
    }

    /**
     * Runs the launcher at a fixed bottom-motor velocity while held, stopping on release.
     * @param rpm Target bottom motor velocity in rotations per minute.
     */
    public Command runVelocityCommand(double rpm) {
        return runEnd(
            () -> runVelocityRPM(rpm),
            this::stop);
    }

    /**
     * Runs at a fixed bottom-wheel surface speed while held, stopping on release.
     * @param metersPerSecond Target surface speed of the 4" bottom wheel.
     */
    public Command runSurfaceSpeedCommand(double metersPerSecond) {
        return runEnd(
            () -> runSurfaceSpeed(metersPerSecond),
            this::stop);
    }

    /**
     * Runs at the dashboard surface-speed setpoint while held, stopping on release.
     * Setpoint and spin ratio are both re-read every cycle, so either can be tuned
     * without re-scheduling.
     */
    public Command runVelocityFromPrefCommand() {
        return runEnd(
            () -> runSurfaceSpeed(manualSurfaceSpeedMPS.get()),
            this::stop);
    }

    /** Immediately stops both rollers. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    // ==================== Telemetry ====================

    private void telemeterize() {
        Logger.recordOutput("HayLauncher/Running", running);

        // Motor space: what the closed loops are chasing
        Logger.recordOutput("HayLauncher/Bottom Target (rpm)", bottomTargetRPM);
        Logger.recordOutput("HayLauncher/Top Target (rpm)", topTargetRPM);
        Logger.recordOutput("HayLauncher/Bottom Velocity (rpm)", cachedBottomVelocityRPM);
        Logger.recordOutput("HayLauncher/Top Velocity (rpm)", cachedTopVelocityRPM);
        Logger.recordOutput("HayLauncher/Bottom Error (rpm)", bottomTargetRPM - cachedBottomVelocityRPM);
        Logger.recordOutput("HayLauncher/Top Error (rpm)", topTargetRPM - cachedTopVelocityRPM);
        Logger.recordOutput("HayLauncher/At Target", atTargetVelocity());

        // Surface space: what the game piece feels
        double bottomSurface = getBottomSurfaceSpeedMPS();
        double topSurface = getTopSurfaceSpeedMPS();
        Logger.recordOutput("HayLauncher/Surface/Bottom (mps)", bottomSurface);
        Logger.recordOutput("HayLauncher/Surface/Top (mps)", topSurface);
        Logger.recordOutput("HayLauncher/Surface/Differential (mps)", bottomSurface - topSurface);
        Logger.recordOutput("HayLauncher/Surface/Estimated Exit (mps)", getEstimatedExitSpeedMPS());
        Logger.recordOutput("HayLauncher/Surface/SpinRatio", spinRatio.get());

        // Health
        Logger.recordOutput("HayLauncher/Bottom Current (A)", bottomMotor.getOutputCurrent());
        Logger.recordOutput("HayLauncher/Top Current (A)", topMotor.getOutputCurrent());
    }
}
