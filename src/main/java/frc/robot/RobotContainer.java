// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.bindings.CommandBinder;
import frc.lib.commands.PathPlannerCommands;
import frc.lib.utils.filters.FilteredJoystick;
import frc.robot.Robot.RobotMode;
import frc.robot.StateHandler.MacroState;
import frc.robot.bindings.SK26StateBinder;
import frc.robot.bindings.SKSwerveBinder;
import frc.robot.bindings.SKTargetPointsBinder;
import frc.robot.bindings.SKVisionBinder;
import frc.robot.subsystems.SubsystemControls;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.vision.SKVision;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Optional<SKSwerve> m_swerveContainer = Optional.empty();
  public Optional<SKVision> m_visionContainer = Optional.empty();
  public Optional<StateHandler> m_stateHandlerContainer = Optional.empty();

  public static Field2d m_field = new Field2d();

  // The list containing all the command binding classes
  public List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();
  LoggedDashboardChooser<Command> autoCommandSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // Creates all subsystems that are on the robot
    configureSubsystems();

    // sets up autos needed for pathplanner
    configurePathPlannerCommands();

    // Configure the trigger bindings
    configureButtonBindings();
  
    if(m_swerveContainer.isPresent()) {
        autoCommandSelector = new LoggedDashboardChooser<>("Select an Auto", AutoBuilder.buildAutoChooser());
    }
  }

  /**
     * Will create all the optional subsystems using the json file in the deploy directory
     */
    private void configureSubsystems()
    {
        File deployDirectory = Filesystem.getDeployDirectory();

        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();

        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser =
                    factory.createParser(new File(deployDirectory, Konstants.SUBSYSTEMFILE));
            SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);

            // The state handler should always be present
            m_stateHandlerContainer = Optional.of(new StateHandler());

            // When robot is simulating and in controlled mode:
            // See turrret simulation from 2026 for what this looks like in practice
            if(Robot.isSimulation() && Robot.Mode == RobotMode.CONTROLLED) {
                if(subsystems.isSwervePresent()) {
                    m_swerveContainer = Optional.of(new SKSwerve());
                }
                if(subsystems.isVisionPresent()) {
                    m_visionContainer = Optional.of(new SKVision(m_swerveContainer));
                }
            }
            // When robot is real or is replaying a real match:
            else {
                if(subsystems.isSwervePresent()) {
                    m_swerveContainer = Optional.of(new SKSwerve());
                }
                if(subsystems.isVisionPresent()) {
                    m_visionContainer = Optional.of(new SKVision(m_swerveContainer));
                }
            }
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }
    }

  /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link FilteredJoystick}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings()
    {
        buttonBinders.add(new SK26StateBinder(m_stateHandlerContainer));
        buttonBinders.add(new SKSwerveBinder(m_swerveContainer));
        buttonBinders.add(new SKTargetPointsBinder());
        buttonBinders.add(new SKVisionBinder(m_visionContainer, m_swerveContainer));
        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }
    }

    private void configurePathPlannerCommands()
    {
        NamedCommands.registerCommands(PathPlannerCommands.getAvailableCommands());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * <p>
     * This method loads the auto when it is called, however, it is recommended
     * to first load your paths/autos when code starts, then return the
     * pre-loaded auto/path.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoCommandSelector.get().withName(autoCommandSelector.getSendableChooser().getSelected());
    }

    public void disabledInit() {
        // Throttle Limelights to reduce CPU load while disabled
        m_visionContainer.ifPresent(vision -> vision.onDisabled());
    }

    public void autonomousInit() {
        if(m_stateHandlerContainer.isPresent()) {
            m_stateHandlerContainer.get().setCurrentState(MacroState.IDLE);
        }
        // Remove Limelight throttling for full performance during auto
        m_visionContainer.ifPresent(vision -> vision.onEnabled());
    }

    public void teleopInit() {
        if(m_stateHandlerContainer.isPresent()) {
            m_stateHandlerContainer.get().setCurrentState(MacroState.IDLE);
        }
        // Remove Limelight throttling for full performance during teleop
        m_visionContainer.ifPresent(vision -> vision.onEnabled());
    }

    public void testPeriodic()
    {
      // Kept this as an example of what should go here.

        // if(m_coral.isPresent())
        // {
        //     m_coral.get().testPeriodic();
        // }
    }
    public void testInit()
    {
    }
}
