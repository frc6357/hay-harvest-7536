package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static frc.robot.Ports.IntakePorts.kIntakePivotMotor;
import static frc.robot.Ports.IntakePorts.kIntakeRollerMotor;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Konstants.IntakeConstants.kIntakeRollerMotorP;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerMotorI;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotMotorD;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotMotorI;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotMotorP;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotSpeed;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerMotorD;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class SKIntake extends SubsystemBase {

    SparkFlex intakeRollerMotor = new SparkFlex(kIntakeRollerMotor.ID, MotorType.kBrushless);
    SparkFlex intakePivotMotor = new SparkFlex(kIntakePivotMotor.ID, MotorType.kBrushless);

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    SparkFlexConfig pivotConfig = new SparkFlexConfig();

    @Getter
    @Setter
    public boolean isIntakeDeployed = false;

    public boolean isIntakeAtTarget = false;

    public SKIntake() {

        rollerConfig.inverted(false);
        rollerConfig.closedLoop.p(kIntakeRollerMotorP).i(kIntakeRollerMotorI).d(kIntakeRollerMotorD);

        pivotConfig.inverted(false);
        pivotConfig.closedLoop.p(kIntakePivotMotorP).i(kIntakePivotMotorI).d(kIntakePivotMotorD);

        intakeRollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakePivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void runIntakeRollers(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void stopIntakeRollers() {
        intakeRollerMotor.set(0);
    }

    public void runIntakePivot() {
        intakePivotMotor.set(kIntakePivotSpeed);
    }

    public void stopIntakePivot() {
        intakePivotMotor.set(0);
    }

    @Override
    public void periodic() {
        logOutputs();
    }

    public void logOutputs() {
        Logger.recordOutput("Intake Roller Motor Speed", intakeRollerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Is Intake Pivot At Target Position", isIntakeAtTarget);
    }
    
}
