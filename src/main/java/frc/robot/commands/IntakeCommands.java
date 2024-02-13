package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Intake;

/**
 * The class for the intake commands.
 */
public final class IntakeCommands {
    /**
     * Gets the command that runs the intake motor forwards.
     * 
     * @return The command that runs the intake motor forwards.
     */
    public static RunCommand getRunIntakeForwardsCommand() {
        return new RunCommand(
            () -> Intake.getInstance().runIntakeForward(), 
            Intake.getInstance()
        );
    }

    /**
     * Gets the command that runs the intake motor backwards.
     * 
     * @return The command that runs the intake motor backwards.
     */
    public static RunCommand getRunIntakeBackwardsCommand() {
        return new RunCommand(
            () -> Intake.getInstance().runIntakeBackward(),
            Intake.getInstance()
        );
    }

    /**
     * Gets the command that stops the intake motor.
     * 
     * @return The commands that stops the intake motor.
     */
    public static InstantCommand getStopIntakeCommand() {
        return new InstantCommand(
            () -> Intake.getInstance().stopIntake(),
            Intake.getInstance()
        );
    }
}
