package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.GeneratedConstants;
import frc.robot.subsystems.drive.SKTargetPoint;

public final class Konstants
{
    public static final class DriveConstants {
        
        public static final LinearVelocity kMaxSpeed = GeneratedConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
        public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(1.75);
        public static final LinearVelocity kMaxSpeedSLOW = kMaxSpeed.times(0.35);
        
        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(1.25); // 3/4 of a rotation per second max angular velocity
        public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2); // 1.5 rotations per second max angular velocity
        public static final AngularVelocity kMaxAngularRateSLOW = kMaxAngularRate.times(0.333); // 1/4 of a rotation per second max angular velocity
        
        public static final LinearAcceleration kMaxTeleopLinAcceleration = kMaxSpeed.div(Seconds.of(0.33));
        public static final AngularAcceleration kMaxTeleopRotAcceleration = kMaxAngularRate.div(Seconds.of(0.33));

        public static final class RotationAligningConstants {
            public static final double kP = 0.85;
            public static final double kI = 0.1;
            public static final double kD = 0.03;

            public static final Rotation2d[] kBumpJumpAngles = new Rotation2d[] {
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(135),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-135)
            };
        }
    }

    // Can be used to make more target points- one dynamic one is all you really need
    public static final class TargetPointConstants {
        public enum TargetPoint {
            kOperatorControlled(
                new SKTargetPoint(new Translation2d(0, 0), "Operator")
            );

            public SKTargetPoint point;

            private TargetPoint(SKTargetPoint point) {
                this.point = point;
            }
        }
    }

    public static final class SwerveConstants
    {
        // swerve chassis width and length in inches 
        public static final double kChassisLength = 27.5;
        public static final double kChassisWidth = 27.5; 
    }

    public static final class AutoConstants
    {
        // PID Constants
        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(6.4, 0.05, 0);
        public static final PIDConstants kRotationPIDConstants    = new PIDConstants(6, 0.4, 0.0);

        public static final PPHolonomicDriveController pathConfig = new PPHolonomicDriveController(kTranslationPIDConstants, kRotationPIDConstants);

        // Tweak if needed- or make a new set of constraints for your specific scenario
        public static final PathConstraints kDefaultPathfindingConstraints = new PathConstraints(
            4.0, 3.65, 
            540, 720, 
            12, false);
    }

    public static final class VisionConstants { // Each limelight has a greek letter name and an individual class for their own set of constants
        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final int kAprilTagPipeline = 0; // Default Apriltag pipeline value for all Limelights

        /* Example:
        public static final class RightLimelight {
            // Network/pipeline values
            public static final String kName = "right-limelight"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = 0.17145; // (z) meters forward of center; negative is backwards
            public static final double kRight = 0.27305; // (x) meters right of center; negative is left
            public static final double kUp = 0.28575; // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 5; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = true;
        }
        */
    }

    /** Constants that are used when defining filters for controllers */
    public static final class OIConstants
    {
        // Controller constraints
        public static final double kJoystickDeadband = 0.15;
    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }

    public static final class IntakeConstants
    {
        public static enum IntakePosition
        {
            /** Set the intake angle to -0.235 intake rotations */
            GROUND(-0.235), // -0.02 with encoder
            /** Set the intake angle to 0.02 intake rotations */
            COMPACTING(0.02),
            /** Set the intake angle to 0 rotations (zero position) */
            STOW(0.0),
            /** Set the intake angle so far back that it guarantees a full stow even with bad PID tuning */
            FULL_STOW(0.1);

            /**
             * The target position for the intake in mechanism rotations (not motor rotations). Positive is up, negative is down.
             */
            public final double rotations;
            IntakePosition(double rotations)
            {
                this.rotations = rotations;
            }
        }

        // PID Constants
        public static final double kPositionerKp = 12.5; //20
        public static final double kPositionerKi = 2.0;
        public static final double kPositionerKd = 0.5;
        public static final double kPositionerKs = 0.5;
        public static final double kPositionerKv = 0.0;
        public static final double kPositionerKa = 0.0;
        public static final double kPositionerKG = 0.0;

        // Positioner voltage limits
        public static final double kPositionerPeakForwardVoltage = 4.0;
        public static final double kPositionerPeakReverseVoltage = -4.0;

        // Positioner Motion Magic configuration
        public static final double kPositionerMMCruiseVelocity = 2.0;      // Rotations per second
        public static final double kPositionerMMAcceleration = 25.0;       // Rotations per second squared
        public static final double kPositionerMMJerk = 50.0;               // Rotations per second cubed
        public static final double kPositionerMMExpoKV = 0.12;
        public static final double kPositionerMMExpoKA = 0.1;

        // Positioner current limits
        public static final double kPositionerSupplyCurrentLimit = 70;
        public static final double kPositionerStatorCurrentLimit = 120;

        // Positioner feedback configuration
        public static final double kPositionerSensorToMechanismRatio = 16.4;
        public static final double kPositionerGainSchedulerErrorThreshold = 0.03;
        public static final double kPositionerPositionTolerance = 0.03;    // Rotations

        // Absolute encoder (CANcoder) configuration
        public static final double kPositionerEncoderOffset = -0.87548828125;         // Rotations (-0.5 to +0.5) — set after measuring zero
        public static final boolean kPositionerEncoderInverted = false;    // Set true if encoder reads backwards
        public static final double kPositionerEncoderGearRatio = 17.76;      // was 3.636363636363
        public static final double kPositionerEncoderDiscontinuityPoint = 1;

        // Intake roller current limits
        public static final double kIntakeSupplyCurrentLimit = 40;
        public static final double kIntakeStatorCurrentLimit = 60;

        // Intake compact command oscillation
        public static final double kIntakeCompactSwitchIntervalSeconds = 0.387;

        public static final double kIntakeMotorSpeed = 0.5;
        public static final double kPositionerMotorSpeed = 0.5;

        public static final double kPositionerMotorMinPosition = 0.5;
        public static final double kPositionMotorMaxPosition = 0.5;

        public static final double kMaxIntakeVoltage = 8.0;

        public static final double kIntakeFullVoltage = -5.5;
        public static final double kIntakeStationaryVoltage = -4.0;
        public static final double kIntakeIdleVoltage = 0.0;

        public static final double kChassisSpeedRollerFF = 1.4; // Volts of output per m/s of velocity in the intake's direction
    }

    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

