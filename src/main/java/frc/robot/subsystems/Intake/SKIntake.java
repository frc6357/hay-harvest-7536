package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Ports.IntakePorts.kIntakeMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKIntake extends SubsystemBase {

    SparkFlex intakeMotor;

    public SKIntake() {

        intakeMotor  = new SparkFlex(kIntakeMotor.ID, MotorType.kBrushless);

    }
    
}
