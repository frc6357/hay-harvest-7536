package frc.robot.commands.IntakeCommands;

import static frc.robot.Konstants.IntakeConstants.kIntakePivotStowed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.SKIntake;

/**
 * Bind with onTrue: stops the rollers and moves the pivot back to the stowed (zero) position.
 * Finishes once the pivot gets there, then the hold command keeps it stowed.
 */
public class StowIntakeCommand extends Command {
    private SKIntake m_intake;

    public StowIntakeCommand(SKIntake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.stopIntakeRollers();
        m_intake.stowIntake();
    }

    @Override
    public void execute() {
        // This command interrupts the hold command, so keep refreshing the pivot setpoint here
        m_intake.setPivotPosition(kIntakePivotStowed);
    }

    @Override
    public boolean isFinished() {
        return m_intake.isPivotNear(kIntakePivotStowed);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
