package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;

/**
 * The class for the intake commands.
 */
public final class IntakeCommands {
    /**
     * Gets the run intake forwards command, which runs
     * the intake forwards.
     * 
     * @return The run intake forwards command.
     */
    public static Command getRunIntakeForwardsCommand() {
        return new InstantCommand(
            () -> Intake.getInstance().runIntakeForwards(), 
            Intake.getInstance());
    }

    /**
     * Gets the run intake backwards command, which runs
     * the intake backwards.
     * 
     * @return The run intake backwards command.
     */
    public static Command getRunIntakeBackwardsCommand() {
        return new InstantCommand(
            () -> Intake.getInstance().runIntakeBackwards(),
            Intake.getInstance());
    }

    /**
     * Gets the stop intake command, which stops
     * the intake.
     * 
     * @return The stop intake command.
     */
    public static Command getStopIntakeCommand() {
        return new InstantCommand(
            () -> Intake.getInstance().stopIntake(),
            Intake.getInstance());
    }
}