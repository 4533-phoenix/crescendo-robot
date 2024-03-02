package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.AmpCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LeftClimb;
import frc.robot.subsystems.RightClimb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Apriltag;

/**
 * The class for setting up the subsystems, buttons, and autonomous commands.
 */
public final class RobotContainer {
    /**
     * The driver controller.
     */
    private static final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_ID);

    /**
     * The manipulator controller.
     */
    private static final XboxController manipulatorController = new XboxController(ControllerConstants.MANIPULATOR_CONTROLLER_ID);

    /**
     * The driver joystick.
     */
    private static final Joystick driverJoystick = new Joystick(JoystickConstants.DRIVER_JOYSTICK_ID);

    /**
     * The manipulator joystick.
     */
    private static final Joystick manipulatorJoystick = new Joystick(JoystickConstants.MANIPULATOR_JOYSTICK_ID);

    /**
     * The auto commands.
     */
    private static final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry(
            AutoConstants.EXIT_AUTO_KEY,
            AutoCommands.followPathAuto(
                AutoConstants.EXIT_AUTO_PATH_FILE_NAME, 
                AutoConstants.EXIT_AUTO_INITIAL_CHASSIS_SPEEDS, 
                AutoConstants.EXIT_AUTO_INITIAL_POSITION.getRotation()
            )
        )
    );

    /**
     * The auto positions corresponding to the auto commands.
     */
    private static final Map<String, Pose2d> autoPositions = Map.ofEntries(
        Map.entry(AutoConstants.EXIT_AUTO_KEY, AutoConstants.EXIT_AUTO_INITIAL_POSITION)
    );

    /**
     * This method registers the controller buttons with their corresponding commands.
     */
    public static void registerButtons() {
        // JoystickButton runSysIDDriveQuasistaticForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_A);
        // runSysIDDriveQuasistaticForwardButton.whileTrue(Robot.sysIDDriveRoutine.quasistatic(Direction.kForward));

        // JoystickButton runSysIDDriveQuasistaticBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_B);
        // runSysIDDriveQuasistaticBackwardButton.whileTrue(Robot.sysIDDriveRoutine.quasistatic(Direction.kReverse));

        // JoystickButton runSysIDDriveDynamicForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_Y);
        // runSysIDDriveDynamicForwardButton.whileTrue(Robot.sysIDDriveRoutine.dynamic(Direction.kForward));

        // JoystickButton runSysIDDriveDynamicBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_X);
        // runSysIDDriveDynamicBackwardButton.whileTrue(Robot.sysIDDriveRoutine.dynamic(Direction.kReverse));

        // JoystickButton runSysIDSteerQuasistaticForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_START);
        // runSysIDSteerQuasistaticForwardButton.whileTrue(Robot.sysIDSteerRoutine.quasistatic(Direction.kForward));

        // JoystickButton runSysIDSteerQuasistaticBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_RB);
        // runSysIDSteerQuasistaticBackwardButton.whileTrue(Robot.sysIDSteerRoutine.quasistatic(Direction.kReverse));

        // JoystickButton runSysIDSteerDynamicForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_LB);
        // runSysIDSteerDynamicForwardButton.whileTrue(Robot.sysIDSteerRoutine.dynamic(Direction.kForward));

        // JoystickButton runSysIDSteerDynamicBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_BACK);
        // runSysIDSteerDynamicBackwardButton.whileTrue(Robot.sysIDSteerRoutine.dynamic(Direction.kReverse));

        Trigger runShooterForwardsTrigger = new Trigger(() -> { return driverController.getRightTriggerAxis() >= ControllerConstants.ANALOG_INPUT_DEADBAND; });
        runShooterForwardsTrigger.whileTrue(ShooterCommands.getRunShooterForwardsCommand());
        runShooterForwardsTrigger.onFalse(ShooterCommands.getStopShooterCommand());

        Trigger runIntakeNoteTrigger = new Trigger(() -> { return driverController.getLeftTriggerAxis() >= ControllerConstants.ANALOG_INPUT_DEADBAND; });
        runIntakeNoteTrigger.whileTrue(ShooterCommands.getIntakeNoteCommand());
        runIntakeNoteTrigger.onFalse(ShooterCommands.stopIntakeNoteCommand());

        // JoystickButton runClimbUpButton = new JoystickButton(manipulatorController, ControllerConstants.BUTTON_RB);
        // runClimbUpButton.whileTrue(ClimbCommands.getRunClimbUpCommand());
        // runClimbUpButton.onFalse(ClimbCommands.getStopClimbCommand());

        // JoystickButton runClimbDownButton = new JoystickButton(manipulatorController, ControllerConstants.BUTTON_LB);
        // runClimbDownButton.whileTrue(ClimbCommands.getRunClimbDownCommand());
        // runClimbDownButton.onFalse(ClimbCommands.getStopClimbCommand());

        // JoystickButton runAmpForwardsButton = new JoystickButton(manipulatorController, ControllerConstants.BUTTON_A);
        // runAmpForwardsButton.whileTrue(AmpCommands.getRunAmpForwardsCommand());
        // runAmpForwardsButton.onFalse(AmpCommands.getStopAmpCommand());

        JoystickButton runAmpDropNoteButton = new JoystickButton(manipulatorController, ControllerConstants.BUTTON_Y);
        runAmpDropNoteButton.whileTrue(AmpCommands.getAmpDropCommand());
        runAmpDropNoteButton.onFalse(AmpCommands.getAmpReceiveCommand());
    }

    /**
     * This method registers the subsystems and their corresponding
     * default commands with the command scheduler.
     */
    public static void registerSubsystems() {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        commandScheduler.registerSubsystem(
            Swerve.getInstance(), 
            Intake.getInstance(),
            Shooter.getInstance(),
            LeftClimb.getInstance(),
            RightClimb.getInstance(),
            Amp.getInstance(),
            Apriltag.getInstance()
        );

        commandScheduler.setDefaultCommand(
            Swerve.getInstance(), 
            SwerveCommands.getDefaultDriveCommand(
                () -> { 
                    return Math.abs(driverController.getLeftY()) >= ControllerConstants.ANALOG_INPUT_DEADBAND
                        ? -driverController.getLeftY()
                        : 0.0;
                },
                () -> {
                    return Math.abs(driverController.getLeftX()) >= ControllerConstants.ANALOG_INPUT_DEADBAND
                        ? -driverController.getLeftX()
                        : 0.0;
                },
                () -> {
                    return Math.abs(driverController.getRightX()) >= ControllerConstants.ANALOG_INPUT_DEADBAND
                        ? -driverController.getRightX()
                        : 0.0;
                }
            )
        );
    }

    /**
     * Gets the auto command corresponding to the given key.
     * 
     * @param key The key corresponding to the auto command.
     * 
     * @return The auto command corresponding to the given key.
     */
    public static Command getAutonomous(String key) {
        return autoCommands.get(key);
    }

    /**
     * Gets the auto position corresponding to the given key.
     * 
     * @param key The key corresponding to the auto position.
     * 
     * @return The auto position corresponding to the given key.
     */
    public static Pose2d getAutonomousPosition(String key) {
        return autoPositions.get(key);
    }
}
