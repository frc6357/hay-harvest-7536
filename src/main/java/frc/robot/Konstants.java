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
        public static final double kIntakeRollerMotorP = 0.0;
        public static final double kIntakeRollerMotorI = 0.0;
        public static final double kIntakeRollerMotorD = 0.0;

        public static final double kIntakePivotMotorP = 0.0;
        public static final double kIntakePivotMotorI = 0.0;
        public static final double kIntakePivotMotorD = 0.0;

        public static final double kIntakePivotStowed = 0.0;
        public static final double kIntakePivotDeployed = 90.0;
        public static final double kIntakePivotSpeed = 0.0;
    }

    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

