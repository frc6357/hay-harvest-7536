package frc.robot.commands.IntakeCommands;

import static frc.robot.Konstants.IntakeConstants.kIntakePivotDeployed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.SKIntake;

/**
 * Bind with onTrue: moves the pivot out to the deployed position without running the rollers.
 * Finishes once the pivot gets there, then the hold command keeps it deployed.
 */
public class DeployIntakeCommand extends Command {
    private SKIntake m_intake;

    public DeployIntakeCommand(SKIntake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.deployIntake();
    }

    @Override
    public void execute() {
        // This command interrupts the hold command, so keep refreshing the pivot setpoint here
        m_intake.setPivotPosition(kIntakePivotDeployed);
    }

    @Override
    public boolean isFinished() {
        return m_intake.isPivotNear(kIntakePivotDeployed);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
