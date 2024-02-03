package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Constants.MotorConstants;

public final class RobotContainer {
    private static final XboxController controller = new XboxController(0);

    private static final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry("Tutorial Auto", AutoCommands.tutorialAuto())
    );

    private static final Map<String, Pose2d> autoPositions = Map.ofEntries(
        Map.entry("Tutorial Auto", new Pose2d())
    );

    public static void registerButtons() {
        // JoystickButton runIntakeForwardButton = new JoystickButton(controller, 1);
        // runIntakeForwardButton.whileTrue(IntakeCommands.getRunIntakeForwardCommand());
        // runIntakeForwardButton.onFalse(IntakeCommands.getStopIntakeCommand());

        // JoystickButton runIntakeBackwardButton = new JoystickButton(controller, 2);
        // runIntakeBackwardButton.whileTrue(IntakeCommands.getRunIntakeBackwardCommand());
        // runIntakeBackwardButton.onFalse(IntakeCommands.getStopIntakeCommand());

        JoystickButton runSysIDDriveQuasistaticForwardButton = new JoystickButton(controller, MotorConstants.BUTTON_A);
        runSysIDDriveQuasistaticForwardButton.whileTrue(Robot.sysIDDriveRoutine.quasistatic(Direction.kForward));

        JoystickButton runSysIDDriveQuasistaticBackwardButton = new JoystickButton(controller, MotorConstants.BUTTON_B);
        runSysIDDriveQuasistaticBackwardButton.whileTrue(Robot.sysIDDriveRoutine.quasistatic(Direction.kReverse));

        JoystickButton runSysIDDriveDynamicForwardButton = new JoystickButton(controller, MotorConstants.BUTTON_Y);
        runSysIDDriveDynamicForwardButton.whileTrue(Robot.sysIDDriveRoutine.dynamic(Direction.kForward));

        JoystickButton runSysIDDriveDynamicBackwardButton = new JoystickButton(controller, MotorConstants.BUTTON_X);
        runSysIDDriveDynamicBackwardButton.whileTrue(Robot.sysIDDriveRoutine.dynamic(Direction.kReverse));

        JoystickButton runSysIDSteerQuasistaticForwardButton = new JoystickButton(controller, MotorConstants.BUTTON_START);
        runSysIDSteerQuasistaticForwardButton.whileTrue(Robot.sysIDSteerRoutine.quasistatic(Direction.kForward));

        JoystickButton runSysIDSteerQuasistaticBackwardButton = new JoystickButton(controller, MotorConstants.BUTTON_RB);
        runSysIDSteerQuasistaticBackwardButton.whileTrue(Robot.sysIDSteerRoutine.quasistatic(Direction.kReverse));

        JoystickButton runSysIDSteerDynamicForwardButton = new JoystickButton(controller, MotorConstants.BUTTON_LB);
        runSysIDSteerDynamicForwardButton.whileTrue(Robot.sysIDSteerRoutine.dynamic(Direction.kForward));

        JoystickButton runSysIDSteerDynamicBackwardButton = new JoystickButton(controller, MotorConstants.BUTTON_BACK);
        runSysIDSteerDynamicBackwardButton.whileTrue(Robot.sysIDSteerRoutine.dynamic(Direction.kReverse));
    }

    public static void registerSubsystems() {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        commandScheduler.registerSubsystem(Swerve.getInstance(), Intake.getInstance());

        // commandScheduler.setDefaultCommand(
        //     Swerve.getInstance(), 
        //     SwerveCommands.getDefaultDriveCommand()
        // );
    }

    public static XboxController getController() {
        return controller;
    }

    public static Command getAutonomous(String key) {
        return autoCommands.get(key);
    }

    public static Pose2d getAutonomousPosition(String key) {
        return autoPositions.get(key);
    }
}
