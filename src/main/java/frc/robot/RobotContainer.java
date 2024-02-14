package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/**
 * The class for setting up the subsystems, buttons, and autonomous commands.
 */
public final class RobotContainer {
    private static final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
    private static final XboxController manipulatorController = new XboxController(ControllerConstants.MANIPULATOR_CONTROLLER_ID);

    private static final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry(
            "Example Auto", 
            AutoCommands.followPathAuto(
                AutoConstants.EXAMPLE_AUTO_PATH_FILE_NAME,
                AutoConstants.EXAMPLE_AUTO_INITIAL_CHASSIS_SPEEDS,
                AutoConstants.EXAMPLE_AUTO_INITIAL_POSITION.getRotation()
            )
        )
    );

    private static final Map<String, Pose2d> autoPositions = Map.ofEntries(
        Map.entry("Example Auto", AutoConstants.EXAMPLE_AUTO_INITIAL_POSITION)
    );

    public static void registerButtons() {
        JoystickButton runSysIDDriveQuasistaticForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_A);
        runSysIDDriveQuasistaticForwardButton.whileTrue(Robot.sysIDDriveRoutine.quasistatic(Direction.kForward));

        JoystickButton runSysIDDriveQuasistaticBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_B);
        runSysIDDriveQuasistaticBackwardButton.whileTrue(Robot.sysIDDriveRoutine.quasistatic(Direction.kReverse));

        JoystickButton runSysIDDriveDynamicForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_Y);
        runSysIDDriveDynamicForwardButton.whileTrue(Robot.sysIDDriveRoutine.dynamic(Direction.kForward));

        JoystickButton runSysIDDriveDynamicBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_X);
        runSysIDDriveDynamicBackwardButton.whileTrue(Robot.sysIDDriveRoutine.dynamic(Direction.kReverse));

        JoystickButton runSysIDSteerQuasistaticForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_START);
        runSysIDSteerQuasistaticForwardButton.whileTrue(Robot.sysIDSteerRoutine.quasistatic(Direction.kForward));

        JoystickButton runSysIDSteerQuasistaticBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_RB);
        runSysIDSteerQuasistaticBackwardButton.whileTrue(Robot.sysIDSteerRoutine.quasistatic(Direction.kReverse));

        JoystickButton runSysIDSteerDynamicForwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_LB);
        runSysIDSteerDynamicForwardButton.whileTrue(Robot.sysIDSteerRoutine.dynamic(Direction.kForward));

        JoystickButton runSysIDSteerDynamicBackwardButton = new JoystickButton(driverController, ControllerConstants.BUTTON_BACK);
        runSysIDSteerDynamicBackwardButton.whileTrue(Robot.sysIDSteerRoutine.dynamic(Direction.kReverse));

        Trigger runRightShooterForwardsTrigger = new Trigger(() -> { return manipulatorController.getRightTriggerAxis() >= 0.05; });
        runRightShooterForwardsTrigger.whileTrue(ShooterCommands.getRunShooterForwardsCommand());
        runRightShooterForwardsTrigger.onFalse(ShooterCommands.getStopShooterCommand());

        Trigger runShooterBackwardsTrigger = new Trigger(() -> { return manipulatorController.getLeftTriggerAxis() >= 0.05; });
        runShooterBackwardsTrigger.whileTrue(ShooterCommands.getRunShooterBackwardsCommand());
        runShooterBackwardsTrigger.onFalse(ShooterCommands.getStopShooterCommand());

        Trigger runIntakeForwardTrigger = new Trigger(() -> { return manipulatorController.getRightTriggerAxis() >= 0.05; });
        runIntakeForwardTrigger.whileTrue(IntakeCommands.getRunIntakeForwardsCommand());
        runIntakeForwardTrigger.onFalse(IntakeCommands.getStopIntakeCommand());

        Trigger runIntakeBackwardTrigger = new Trigger(() -> { return manipulatorController.getLeftTriggerAxis() >= 0.05; });
        runIntakeBackwardTrigger.whileTrue(IntakeCommands.getRunIntakeBackwardsCommand());
        runIntakeBackwardTrigger.onFalse(IntakeCommands.getStopIntakeCommand());

        JoystickButton runClimbForwardsButton = new JoystickButton(manipulatorController, ControllerConstants.BUTTON_RB);
        runClimbForwardsButton.whileTrue(ClimbCommands.getRunClimbUpwardsCommand());
        runClimbForwardsButton.onFalse(ClimbCommands.getStopClimbCommand());

        JoystickButton runClimbBackwardsButton = new JoystickButton(manipulatorController, ControllerConstants.BUTTON_LB);
        runClimbBackwardsButton.whileTrue(ClimbCommands.getRunClimbDownwardsCommand());
        runClimbBackwardsButton.onFalse(ClimbCommands.getStopClimbCommand());
    }

    public static void registerSubsystems() {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        commandScheduler.registerSubsystem(Swerve.getInstance(), Intake.getInstance());

        // commandScheduler.setDefaultCommand(
        //     Swerve.getInstance(), 
        //     SwerveCommands.getDefaultDriveCommand(
        //         () -> { 
        //             return Math.abs(driverController.getLeftX()) >= ControllerConstants.JOYSTICK_DEADBAND
        //                 ? driverController.getLeftX()
        //                 : 0.0;
        //         },
        //         () -> {
        //             return Math.abs(driverController.getLeftY()) >= ControllerConstants.JOYSTICK_DEADBAND
        //                 ? driverController.getLeftY()
        //                 : 0.0;
        //         },
        //         () -> {
        //             return Math.abs(driverController.getRightX()) >= ControllerConstants.JOYSTICK_DEADBAND
        //                 ? driverController.getRightX()
        //                 : 0.0;
        //         }
        //     )
        // );
    }

    public static Command getAutonomous(String key) {
        return autoCommands.get(key);
    }

    public static Pose2d getAutonomousPosition(String key) {
        return autoPositions.get(key);
    }
}
