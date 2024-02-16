package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Amp;

/**
 * The class for the amp commands.
 */
public final class AmpCommands {
    /**
     * Gets the command that runs the amp motor forwards.
     * 
     * @return The command that runs the amp motor forwards.
     */
    public static Command getRunAmpForwardsCommand() {
        return new RunCommand(
            () -> Amp.getInstance().runAmpForwards(),
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that runs the amp motor backwards.
     * 
     * @return The command that runs the amp motor backwards.
     */
    public static Command getRunAmpBackwardsCommand() {
        return new RunCommand(
            () -> Amp.getInstance().runAmpBackwards(), 
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that stops the amp motor.
     * 
     * @return The command that stops the amp motor.
     */
    public static Command getStopAmpCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().stopAmp(),
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that sets the amp subsystem position
     * to the receive position.
     * 
     * @return The command that sets the amp subsystem position
     * to the receive position.
     */
    public static Command getAmpReceiveCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> Amp.getInstance().runAmpForwards(), 
            (isFinished) -> Amp.getInstance().stopAmp(), 
            () -> Amp.getInstance().getReceiveLimitSwitchPressed(), 
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that sets the amp subsystem position
     * to the drop position.
     * 
     * @return The command that sets the amp subsystem position
     * to the drop position.
     */
    public static Command getAmpDropCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> Amp.getInstance().runAmpBackwards(),
            (isFinished) -> Amp.getInstance().stopAmp(),
            () -> Amp.getInstance().getDropLimitSwitchPressed(),
            Amp.getInstance()
        );
    }
}
