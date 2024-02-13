package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climb;

/**
 * The class for the climb commands.
 */
public class ClimbCommands {
    /**
     * Gets the command that runs the climb motors down.
     * 
     * @return The command that runs the climb motors down.
     */
    public static RunCommand getRunClimbDownwardsCommand() {
        return new RunCommand(
            () -> Climb.getInstance().runClimbDown(),
            Climb.getInstance()
        );
    }

    /**
     * Gets the command that runs the climb motors up.
     * 
     * @return The command that runs the climb motors up.
     */
    public static RunCommand getRunClimbUpwardsCommand() {
        return new RunCommand(
            () -> Climb.getInstance().runClimbUp(),
            Climb.getInstance()
        );
    }

    /**
     * Gets the command that stops the climb motors.
     * 
     * @return The command that stops the climb motors.
     */
    public static InstantCommand getStopClimbCommand() {
        return new InstantCommand(
            () -> Climb.getInstance().stopClimb(),
            Climb.getInstance()
        );
    }
}
