package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. It is used to determine which subsystems are present on the robot, and can be used to
 * enable/disable certain features or commands based on the presence of subsystems.
 */
public class SubsystemControls
{

    private final boolean swerve;
    private final boolean vision;
    private final boolean launcher;

    public SubsystemControls(
        @JsonProperty(required = true, value = "swerve")      boolean swerve,
        @JsonProperty(required = true, value = "vision")      boolean vision,
        @JsonProperty(required = true, value = "launcher")    boolean launcher
    )

    {
        this.swerve = swerve;
        this.vision = vision;
        this.launcher = launcher;
    }


    public boolean isSwervePresent() {
        return swerve;
    }
    public boolean isVisionPresent() {
        return vision;
    }
    public boolean isLauncherPresent() {
        return launcher;
    }
}
