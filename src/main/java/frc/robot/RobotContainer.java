package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public final class RobotContainer {
    private static final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
    private static final XboxController manipulatorController = new XboxController(ControllerConstants.MANIPULATOR_CONTROLLER_ID);

    private static final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry("Tutorial Auto", AutoCommands.tutorialAuto())
    );

    private static final Map<String, Pose2d> autoPositions = Map.ofEntries(
        Map.entry("Tutorial Auto", new Pose2d())
    );

    public static void registerButtons() {
        // JoystickButton runIntakeForwardButton = new JoystickButton(driverController, 1);
        // runIntakeForwardButton.whileTrue(IntakeCommands.getRunIntakeForwardCommand());
        // runIntakeForwardButton.onFalse(IntakeCommands.getStopIntakeCommand());

        // JoystickButton runIntakeBackwardButton = new JoystickButton(driverController, 2);
        // runIntakeBackwardButton.whileTrue(IntakeCommands.getRunIntakeBackwardCommand());
        // runIntakeBackwardButton.onFalse(IntakeCommands.getStopIntakeCommand());

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

        Trigger runRightShooterForwardsTrigger = new Trigger(() -> { return driverController.getRightTriggerAxis() >= 0.05; });
        runRightShooterForwardsTrigger.whileTrue(ShooterCommands.getRunShooterForwardsCommand());
        runRightShooterForwardsTrigger.onFalse(ShooterCommands.getStopShooterCommand());

        Trigger runShooterBackwardsTrigger = new Trigger(() -> { return driverController.getLeftTriggerAxis() >= 0.05; });
        runShooterBackwardsTrigger.whileTrue(ShooterCommands.getRunShooterBackwardsCommand());
        runShooterBackwardsTrigger.onFalse(ShooterCommands.getStopShooterCommand());
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
