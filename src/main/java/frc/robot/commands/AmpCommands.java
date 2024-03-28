package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.AmpConstants.AMP_MOVEMENT_DIRECTION;
import frc.robot.Constants.AmpConstants.AMP_POSITION;
import frc.robot.subsystems.Amp;

/**
 * The class for the amp commands.
 */
public final class AmpCommands {
    /**
     * Gets the run amp forwards command, which runs the 
     * amp subsystem forwards.
     * 
     * @return The run amp forwards command.
     */
    public static Command getRunAmpForwardsCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().runAmpForwards(),
            Amp.getInstance());
    }

    /**
     * Gets the run amp backwards command, which runs the
     * amp subsystem backwards.
     * 
     * @return The run amp backwards command.
     */
    public static Command getRunAmpBackwardsCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().runAmpBackwards(),
            Amp.getInstance());
    }

    /**
     * Gets the stop amp command, which stops the
     * amp subsystem.
     * 
     * @return The stop amp command.
     */
    public static Command getStopAmpCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().stopAmp(),
            Amp.getInstance());
    }

    /**
     * Gets the amp receive command, which sets the amp subsystem 
     * position to the receive position. Delay added to allow the amp
     * subsystem position to be updated after the amp
     * subsystem stops moving in the event that the command
     * is canceled abruptly.
     * 
     * @return The amp receive command.
     */
    public static Command getAmpReceiveCommand() {
        return new FunctionalCommand(
            () -> Amp.getInstance().setAmpMovementDirection(
                    AMP_MOVEMENT_DIRECTION.TOWARDS_RECEIVE_POSITION),
            () -> Amp.getInstance().runAmpBackwards(), 
            (isFinished) -> Amp.getInstance().stopAmp(), 
            () -> Amp.getInstance().isAmpLimitSwitchAtPosition(
                    AMP_POSITION.RECEIVE_POSITION)
                && Amp.getInstance().isAmpLimitSwitchAtMagnet(), 
            Amp.getInstance())
                .andThen(new WaitCommand(AmpConstants.AMP_POSITION_DELAY));
    }

    /**
     * Gets the amp drop command, which sets the amp subsystem 
     * position to the drop position. Delay added to allow the amp
     * subsystem position to be updated after the amp
     * subsystem stops moving in the event that the command
     * is canceled abruptly.
     * 
     * @return The amp drop command.
     */
    public static Command getAmpDropCommand() {
        return new FunctionalCommand(
            () -> Amp.getInstance().setAmpMovementDirection(
                AMP_MOVEMENT_DIRECTION.TOWARDS_DROP_POSITION), 
            () -> Amp.getInstance().runAmpForwards(), 
            (isFinished) -> Amp.getInstance().stopAmp(), 
            () -> Amp.getInstance().isAmpLimitSwitchAtPosition(
                    AMP_POSITION.DROP_POSITION)
                && Amp.getInstance().isAmpLimitSwitchAtMagnet(), 
            Amp.getInstance())
                .andThen(new WaitCommand(AmpConstants.AMP_POSITION_DELAY));
    }
}