package frc.robot.bindings;

import java.util.Optional;

import frc.lib.bindings.CommandBinder;
import frc.robot.commands.IntakeCommands.IntakeButtonCommand;
import frc.robot.commands.IntakeCommands.StowIntakeCommand;
import frc.robot.subsystems.Intake.SKIntake;

import static frc.robot.Ports.OperatorPorts.kLBbutton;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kXbutton;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKIntakeBinder implements CommandBinder {
    Optional<SKIntake> m_intakeContainer;

    Trigger runIntake;
    Trigger stowIntake;
    Trigger ejectIntake;

    public SKIntakeBinder(Optional<SKIntake> m_intakeContainer) {
        this.m_intakeContainer = m_intakeContainer;

        this.runIntake = kLTrigger.button;
        this.stowIntake = kLBbutton.button;
        this.ejectIntake = kXbutton.button;
    }

    public void bindButtons() {
        if(m_intakeContainer.isPresent()) {
            SKIntake m_intake = m_intakeContainer.get();

            // Hold to deploy the intake and run the rollers in
            runIntake.whileTrue(new IntakeButtonCommand(m_intake).withName("IntakeButton"));

            // Press to stop the rollers and bring the intake back to stowed
            stowIntake.onTrue(new StowIntakeCommand(m_intake).withName("StowIntake"));

            // Hold to spit a jammed ball back out (pivot stays wherever it is)
            ejectIntake.whileTrue(m_intake.runRollersEjectCommand());
        }
    }
}
