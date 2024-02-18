package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climb;

/**
 * The class for the climb commands.
 */
public class ClimbCommands {
    /**
     * Gets the command that runs the climb subsystem up.
     * 
     * @return The command that runs the climb subsystem up.
     */
    public static RunCommand getRunClimbUpCommand() {
        return new RunCommand(
            () -> Climb.getInstance().runClimbUp(),
            Climb.getInstance()
        );
    }
    
    /**
     * Gets the command that runs the climb subsystem down.
     * 
     * @return The command that runs the climb subsystem down.
     */
    public static RunCommand getRunClimbDownCommand() {
        return new RunCommand(
            () -> Climb.getInstance().runClimbDown(),
            Climb.getInstance()
        );
    }

    /**
     * Gets the command that stops the climb subsystem.
     * 
     * @return The command that stops the climb subsystem.
     */
    public static InstantCommand getStopClimbCommand() {
        return new InstantCommand(
            () -> Climb.getInstance().stopClimb(),
            Climb.getInstance()
        );
    }
}