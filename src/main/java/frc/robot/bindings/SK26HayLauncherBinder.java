package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kRTrigger;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.shooter.SK26HayLauncher;

public class SK26HayLauncherBinder implements CommandBinder {

    Optional<SK26HayLauncher> launcherContainer;

    Trigger score;

    public SK26HayLauncherBinder(Optional<SK26HayLauncher> launcherContainer) {
        this.launcherContainer = launcherContainer;

        this.score = kRTrigger.button;
    }

    @Override
    public void bindButtons() {
        if (launcherContainer.isPresent()) {
            SK26HayLauncher launcher = launcherContainer.get();

            // Hold operator right trigger to spin up, release to coast down.
            // Speed comes from the HayLauncher/ManualSurfaceSpeed (mps) preference,
            // re-read every cycle, so it can be swept live while characterizing.
            // Swap to a distance-based supplier once a speed map exists.
            score.whileTrue(
                launcher.runVelocityFromPrefCommand().withName("HayLauncherScore"));
        }
    }
}
