package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Intake;

/**
 * The class for the intake commands.
 */
public final class IntakeCommands {
    /**
     * Gets the command that runs the intake subsystem forwards.
     * 
     * @return The command that runs the intake subsystem forwards.
     */
    public static RunCommand getRunIntakeForwardsCommand() {
        return new RunCommand(
            () -> Intake.getInstance().runIntakeForwards(), 
            Intake.getInstance()
        );
    }

    /**
     * Gets the command that runs the intake subsystem backwards.
     * 
     * @return The command that runs the intake subsystem backwards.
     */
    public static RunCommand getRunIntakeBackwardsCommand() {
        return new RunCommand(
            () -> Intake.getInstance().runIntakeBackwards(),
            Intake.getInstance()
        );
    }

    /**
     * Gets the command that stops the intake subsystem.
     * 
     * @return The commands that stops the intake subsystem.
     */
    public static InstantCommand getStopIntakeCommand() {
        return new InstantCommand(
            () -> Intake.getInstance().stopIntake(),
            Intake.getInstance()
        );
    }
}