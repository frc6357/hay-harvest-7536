package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static frc.robot.Ports.IntakePorts.kIntakeMotor;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Konstants.IntakeConstants.kIntakeMotorP;
import static frc.robot.Konstants.IntakeConstants.kIntakeMotorI;
import static frc.robot.Konstants.IntakeConstants.kIntakeMotorD;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKIntake extends SubsystemBase {

    SparkFlex intakeMotor  = new SparkFlex(kIntakeMotor.ID, MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();

    public SKIntake() {

        config.inverted(false);
        config.closedLoop
        .p(kIntakeMotorP)
        .i(kIntakeMotorI)
        .d(kIntakeMotorD);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        logOutputs();
    }

    public void logOutputs() {
        Logger.recordOutput("Intake Roller Motor Speed", intakeMotor.getEncoder().getVelocity());
    }
    
}
