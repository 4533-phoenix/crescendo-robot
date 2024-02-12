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
    public static Command getRunAmpMotorForwardsCommand() {
        return new RunCommand(
            () -> Amp.getInstance().runAmpMotorForwards(),
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that runs the amp motor backwards.
     * 
     * @return The command that runs the amp motor backwards.
     */
    public static Command getRunAmpMotorBackwardsCommand() {
        return new RunCommand(
            () -> Amp.getInstance().runAmpMotorBackwards(), 
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that stops the amp motor.
     * 
     * @return The command that stops the amp motor.
     */
    public static Command getStopAmpMotorCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().stopAmpMotor(),
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
    public static Command getAmpMotorReceiveCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> Amp.getInstance().runAmpMotorForwards(), 
            (isFinished) -> Amp.getInstance().stopAmpMotor(), 
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
    public static Command getAmpMotorDropCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> Amp.getInstance().runAmpMotorBackwards(),
            (isFinished) -> Amp.getInstance().stopAmpMotor(),
            () -> Amp.getInstance().getDropLimitSwitchPressed(),
            Amp.getInstance()
        );
    }
}
