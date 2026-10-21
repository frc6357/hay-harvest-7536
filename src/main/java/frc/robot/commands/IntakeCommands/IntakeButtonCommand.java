package frc.robot.commands.IntakeCommands;

import static frc.robot.Konstants.IntakeConstants.kIntakePivotDeployed;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerIntakeVolts;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollerStartTolerance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.SKIntake;

/**
 * Bind with whileTrue: deploys the intake (if it isn't already), then runs the rollers inward
 * once the pivot reaches the deployed position. Releasing the button stops the rollers and
 * leaves the intake deployed (the hold command takes over).
 */
public class IntakeButtonCommand extends Command {
    private SKIntake m_intake;
    private boolean reachedDeployed;

    public IntakeButtonCommand(SKIntake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        reachedDeployed = false;
        m_intake.deployIntake();
    }

    @Override
    public void execute() {
        // This command interrupts the hold command, so keep refreshing the pivot setpoint here
        m_intake.setPivotPosition(kIntakePivotDeployed);

        // Start rolling once the pivot is close enough (looser than the PID tolerance so a little sag doesn't block it),
        // then keep rolling even if it bounces back out
        if (!reachedDeployed && Math.abs(m_intake.getPivotPosition() - kIntakePivotDeployed) <= kIntakeRollerStartTolerance) {
            reachedDeployed = true;
        }

        if (reachedDeployed) {
            m_intake.runIntakeRollers(kIntakeRollerIntakeVolts);
        } else {
            m_intake.stopIntakeRollers();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntakeRollers();
    }
}
