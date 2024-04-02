package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The class for the shooter commands.
 */
public class ShooterCommands {
    /**
     * Gets the run lift forwards command, which runs
     * the lift forwards.
     * 
     * @return The run lift forwards command.
     */
    public static Command getRunLiftForwardsCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().runLiftForwards(),
            Shooter.getInstance());
    }

    /**
     * Gets the run lift backwards command, which runs
     * the lift backwards.
     * 
     * @return The run lift backwards command.
     */
    public static Command getRunLiftBackwardsCommmand() {
        return new InstantCommand(
            () -> Shooter.getInstance().runLiftBackwards(), 
            Shooter.getInstance());
    }

    /**
     * Gets the stop lift command, which stops
     * the lift.
     * 
     * @return The stop lift command.
     */
    public static Command getStopLiftCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopLift(),
            Shooter.getInstance());
    }

    /**
     * Gets the run shooter forwards command, which runs
     * the shooter forwards.
     * 
     * @return The run shooter forwards command.
     */
    public static Command getRunShooterForwardsCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().runShooterForwards(),
            Shooter.getInstance());
    }

    /**
     * Gets the run shooter backwards command, which runs
     * the shooter backwards.
     * 
     * @return The run shooter backwards command.
     */
    public static Command getRunShooterBackwardsCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().runShooterBackwards(),
            Shooter.getInstance());
    }

    /**
     * Gets the stop shooter command, which stops
     * the shooter.
     * 
     * @return The stop shooter command.
     */
    public static Command getStopShooterCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopShooter(),
            Shooter.getInstance());
    }

    /**
     * Gets the run shooter and lift forwards command, which runs
     * the shooter and lift forwards.
     * 
     * @return The run shooter and lift forwards command.
     */
    public static Command getRunShooterAndLiftForwardsCommand() {
        return new InstantCommand(
            () -> {
                Shooter.getInstance().runShooterForwards();
                Shooter.getInstance().runLiftForwards();
            },
            Shooter.getInstance());
    }

    /**
     * Gets the shoot note command, which shoots
     * a note.
     * 
     * @return The shoot note command.
     */
    public static Command getShootNoteCommand() {
        return new SequentialCommandGroup(
            getRunShooterForwardsCommand(),
            new WaitUntilCommand(
                () -> Shooter.getInstance().isShooterReady()),
            getRunShooterAndLiftForwardsCommand(),
            new WaitCommand(0.4),
            getStopShooterCommand(),
            getStopLiftCommand());
    }

    /**
     * Gets the stop shoot note command, which
     * stops shooting a note. Used if the shoot 
     * note button is released before the shoot 
     * note command finishes.
     * 
     * @return The stop shoot note command.
     */
    public static Command getStopShootNoteCommand() {
        return new InstantCommand(
            () -> {
                Shooter.getInstance().stopShooter();
                Shooter.getInstance().stopLift();
            },
            Shooter.getInstance());
    }

    /**
     * Gets the eject note command, which ejects
     * a note.
     * 
     * @return The eject note command.
     */
    public static Command getEjectNoteCommand() {
        return new InstantCommand(
            () -> {
                Intake.getInstance().runIntakeBackwards();
                Shooter.getInstance().runLiftBackwards();
            },
            Intake.getInstance(),
            Shooter.getInstance());
    }

    /**
     * Gets the stop eject note command, which stops
     * ejecting a note.
     * 
     * @return The stop eject note command.
     */
    public static Command getStopEjectNoteCommand() {
        return new InstantCommand(
            () -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();
            },
            Intake.getInstance(),
            Shooter.getInstance());
    }

    /**
     * Gets the intake note command, which intakes
     * a note. Stops intaking a note when the shooter
     * limit switch is pressed.
     * 
     * @return The intake note command.
     */
    public static Command getIntakeNoteCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                Intake.getInstance().runIntakeForwards();
                Shooter.getInstance().runLiftForwards();
            },
            (isFinished) -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();
            },
            () -> Shooter.getInstance().isLimitSwitchPressed(),
            Intake.getInstance(),
            Shooter.getInstance());
    }

    /**
     * Gets the stop intake note command, which stops 
     * intaking a note. Used if the intake note button 
     * is released before the intake note command is finished.
     * 
     * @return The stop intake note command.
     */
    public static Command stopIntakeNoteCommand() {
        return new InstantCommand(
            () -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();
            },
            Intake.getInstance(),
            Shooter.getInstance());
    }
}
