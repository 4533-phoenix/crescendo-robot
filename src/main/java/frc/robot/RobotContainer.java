package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The class for setting up the subsystems, 
 * buttons, and autonomous commands.
 */
public final class RobotContainer {
    /**
     * The driver controller.
     */
    private static final XboxController driverController = 
        new XboxController(ControllerConstants.DRIVER_CONTROLLER_ID);

    /**
     * The manipulator controller.
     */
    private static final XboxController manipulatorController = 
        new XboxController(ControllerConstants.MANIPULATOR_CONTROLLER_ID);

    /**
     * This method registers the controller buttons 
     * with their corresponding commands.
     */
    public static void registerButtons() {
        /*
         * Create the run SysID drive quasistatic forwards button on the A button
         * of the driver controller.
         */
        // JoystickButton runSysIDDriveQuasistaticForwardsButton = 
        //     new JoystickButton(driverController, ControllerConstants.BUTTON_A);

        /*
         * While the run SysID drive quasistatic forwards button is pressed,
         * run the SysID drive routine quasistatic forwards command.
         */
        // runSysIDDriveQuasistaticForwardsButton.whileTrue(
        //     Robot.sysIDDriveRoutine.quasistatic(Direction.kForward));

        /*
         * Create the run SysID drive quasistatic backwards button on the A button
         * of the driver controller.
         */
        // JoystickButton runSysIDDriveQuasistaticBackwardsButton = 
        //     new JoystickButton(driverController, ControllerConstants.BUTTON_B);

        /*
         * While the run SysID drive quasistatic backwards button is pressed,
         * run the SysID drive routine quasistatic backwards command.
         */
        // runSysIDDriveQuasistaticBackwardsButton.whileTrue(
        //     Robot.sysIDDriveRoutine.quasistatic(Direction.kReverse));
        
        /*
         * Create the run SysID drive dynamic forwards button on the Y button
         * of the driver controller.
         */
        // JoystickButton runSysIDDriveDynamicForwardsButton = 
        //     new JoystickButton(driverController, ControllerConstants.BUTTON_Y);

        /*
         * While the SysID drive dynamic forwards button is pressed,
         * run the SysID drive routine dynamic forwards command.
         */
        // runSysIDDriveDynamicForwardsButton.whileTrue(
        //     Robot.sysIDDriveRoutine.dynamic(Direction.kForward));

        /*
         * Create the run SysID drive dynamic backwards button on the X button
         * of the driver controller.
         */
        // JoystickButton runSysIDDriveDynamicBackwardsButton = 
        //     new JoystickButton(driverController, ControllerConstants.BUTTON_X);

        /*
         * While the run SysID drive dynamic backwards button is pressed,
         * run the SysID drive routine dynamic backwards command.
         */
        // runSysIDDriveDynamicBackwardsButton.whileTrue(
        //     Robot.sysIDDriveRoutine.dynamic(Direction.kReverse));

        /*
         * Create the run intake note trigger on the left trigger
         * of the driver controller.
         */
        Trigger runIntakeNoteTrigger = 
            new Trigger(
                () -> { 
                    return driverController.getLeftTriggerAxis()
                        >= ControllerConstants.ANALOG_INPUT_DEADBAND;
                });
        
        /*
         * While the run intake note trigger is pressed,
         * run the intake note command.
         */
        runIntakeNoteTrigger.whileTrue(
            ShooterCommands.getIntakeNoteCommand());

        /*
         * When the run intake note trigger is released,
         * run the stop intake note command.
         */
        runIntakeNoteTrigger.onFalse(
            ShooterCommands.stopIntakeNoteCommand());
        
        /*
         * Create the run intake backwards button on the RB button
         * of the driver controller.
         */
        JoystickButton runIntakeBackwardsButton = 
            new JoystickButton(
                driverController, 
                ControllerConstants.BUTTON_RB);
        
        /*
         * While the run intake backwards button is pressed,
         * run the eject note command.
         */
        runIntakeBackwardsButton.whileTrue(
            ShooterCommands.getEjectNoteCommand());
        
        /*
         * When the run intake backwards button is released,
         * run the stop eject note command.
         */
        runIntakeBackwardsButton.onFalse(
            ShooterCommands.getStopEjectNoteCommand());
        
        /*
         * Create the run shoot note trigger on the right trigger
         * of the driver controller.
         */
        Trigger runShootNoteTrigger = 
            new Trigger(
                () -> { 
                    return driverController.getRightTriggerAxis()
                        >= ControllerConstants.ANALOG_INPUT_DEADBAND;
                });
        
        /*
         * While the run shoot note trigger is pressed,
         * run the shoot note command.
         */
        runShootNoteTrigger.whileTrue(
            ShooterCommands.getShootNoteCommand());

        /*
         * When the run shoot note trigger is released,
         * run the stop shoot note command.
         */
        runShootNoteTrigger.onFalse(
            ShooterCommands.getStopShootNoteCommand());
        
        /*
         * Create the run climb up button on the RB button
         * of the manipulator controller.
         */
        JoystickButton runClimbUpButton = 
            new JoystickButton(
                manipulatorController, 
                ControllerConstants.BUTTON_RB);
        
        /*
         * While the run climb up button is pressed,
         * run the run climb up command.
         */
        runClimbUpButton.whileTrue(
            ClimbCommands.getRunClimbUpCommand());
        
        /*
         * When the run climb up button is released,
         * run the stop climb command.
         */
        runClimbUpButton.onFalse(
            ClimbCommands.getStopClimbCommand());
        
        /*
         * Create the run climb down button on the LB button
         * of the manipulator controller.
         */
        JoystickButton runClimbDownButton = 
            new JoystickButton(
                manipulatorController, 
                ControllerConstants.BUTTON_LB);
        
        /*
         * While the run climb down button is pressed,
         * run the run climb down command.
         */
        runClimbDownButton.whileTrue(
            ClimbCommands.getRunClimbDownCommand());

        /*
         * When the run climb down button is released,
         * run the stop climb command.
         */
        runClimbDownButton.onFalse(
            ClimbCommands.getStopClimbCommand());
        
        /*
         * Create the run amp drop note button on the Y button
         * of the manipulator controller.
         */
        JoystickButton runAmpDropNoteButton = 
            new JoystickButton(
                manipulatorController, 
                ControllerConstants.BUTTON_Y);
        
        /*
         * While the run amp drop note button is pressed,
         * run the amp drop command.
         */
        runAmpDropNoteButton.whileTrue(
            AmpCommands.getAmpDropCommand());
        
        /*
         * When the run amp drop note button is released,
         * run the amp receive command.
         */
        runAmpDropNoteButton.onFalse(
            AmpCommands.getAmpReceiveCommand());
        
        /*
         * Create the run set slow mode button on the LB button
         * of the driver controller.
         */
        JoystickButton runSetSlowModeButton = 
            new JoystickButton(
                driverController, 
                ControllerConstants.BUTTON_LB);
        
        /*
         * While the run set slow mode button is pressed,
         * run the set slow mode command to set slow mode
         * to enabled.
         */
        runSetSlowModeButton.whileTrue(
            SwerveCommands.getSetSlowModeCommand(true));

        /*
         * When the run set slow mode button is released,
         * run the set slow mode command to set slow mode
         * to disabled.
         */
        runSetSlowModeButton.onFalse(
            SwerveCommands.getSetSlowModeCommand(false));
        
        /*
         * Create the run toggle robot relative mode button 
         * on the start button of the driver controller.
         */
        JoystickButton runToggleRobotRelativeModeButton = 
            new JoystickButton(
                driverController, 
                ControllerConstants.BUTTON_START);
        
        /*
         * When the run toggle robot relative mode button is pressed,
         * run the toggle robot relative mode command.
         */
        runToggleRobotRelativeModeButton.onTrue(
            SwerveCommands.getToggleRobotRelativeModeCommand());
        
        /*
         * Create the run track and acquire note button on the Y button
         * of the driver controller.
         */
        JoystickButton runTrackAndAcquireNoteButton = 
            new JoystickButton(
                driverController, 
                ControllerConstants.BUTTON_Y);
        
        /*
         * While the run track and acquire note button is pressed,
         * run the track and acquire note command.
         */
        runTrackAndAcquireNoteButton.whileTrue(
            SwerveCommands.getTrackAndAcquireNoteCommand());
        
        /*
         * When the run track and acquire note button is released,
         * run the stop track and acquire note command and then
         * run the intake note command, which is done to intake
         * the note up to the shooter limit switch after the track 
         * and acquire note command has run.
         */
        runTrackAndAcquireNoteButton.onFalse(
            SwerveCommands.getStopTrackAndAcquireNoteCommand()
                .andThen(ShooterCommands.getIntakeNoteCommand()));
        
        /*
         * Create the run drive to point of interest button on the A button
         * of the driver controller.
         */
        JoystickButton runDriveToPointOfInterestButton = 
            new JoystickButton(
                driverController, 
                ControllerConstants.BUTTON_A);
        
        /*
         * While the run drive to point of interest button is pressed,
         * run the drive to point of interest command.
         */
        runDriveToPointOfInterestButton.whileTrue(
            AutoCommands.getDriveToPointOfInterestCommand());

        /*
         * When the run drive to point of interest button is released,
         * run the stop swerve command.
         */
        runDriveToPointOfInterestButton.onFalse(
            SwerveCommands.getStopSwerveCommand());
        
        /*
         * Create the run lock wheels button on the B button
         * of the manipulator controller.
         */
        JoystickButton runLockWheelsButton =
            new JoystickButton(
                driverController, 
                ControllerConstants.BUTTON_B);
        
        /*
         * While the run lock wheels button is pressed,
         * run the lock wheels command.
         */
        runLockWheelsButton.whileTrue(
            SwerveCommands.getLockWheelsCommand());
        
        /*
         * Create the run climb down button on the LB button
         * of the manipulator controller.
         */
        JoystickButton runLimelightFlashButton = 
            new JoystickButton(
                manipulatorController, 
                ControllerConstants.BUTTON_B);
        
        /*
         * While the run climb down button is pressed,
         * run the run climb down command.
         */
        runLimelightFlashButton.whileTrue(
            LimelightCommands.getStartFlashLimelightCommand());

        /*
         * When the run climb down button is released,
         * run the stop climb command.
         */
        runLimelightFlashButton.onFalse(
            LimelightCommands.getStopFlashLimelightCommand());
    }

    /**
     * This method registers the subsystems and their corresponding
     * default commands with the command scheduler.
     */
    public static void registerSubsystems() {
        // Get the command scheduler.
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Register the subsystems with the command scheduler.
        commandScheduler.registerSubsystem(
            Swerve.getInstance(), 
            Intake.getInstance(),
            Shooter.getInstance(),
            LeftClimb.getInstance(),
            RightClimb.getInstance(),
            Amp.getInstance(),
            Apriltag.getInstance());

        // Register the intake not command with the path planner commands.
        NamedCommands.registerCommand(
            AutoConstants.INTAKE_NOTE_COMMAND, 
            ShooterCommands.getIntakeNoteCommand());

        // Register the default drive command with the swerve drive subsystem.
        commandScheduler.setDefaultCommand(
            Swerve.getInstance(), 
            SwerveCommands.getDefaultDriveCommand(
                () -> { 
                    return Math.abs(driverController.getLeftY()) 
                            >= ControllerConstants.ANALOG_INPUT_DEADBAND
                        ? -driverController.getLeftY()
                        : 0.0;
                },
                () -> {
                    return Math.abs(driverController.getLeftX()) 
                            >= ControllerConstants.ANALOG_INPUT_DEADBAND
                        ? -driverController.getLeftX()
                        : 0.0;
                },
                () -> {
                    return Math.abs(driverController.getRightX()) 
                            >= ControllerConstants.ANALOG_INPUT_DEADBAND
                        ? -driverController.getRightX()
                        : 0.0;
                }));
    }
}
