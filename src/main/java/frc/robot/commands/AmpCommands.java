package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
     * Gets the command that runs the amp subsystem forwards.
     * 
     * @return The command that runs the amp subsystem forwards.
     */
    public static Command getRunAmpForwardsCommand() {
        return new RunCommand(
            () -> Amp.getInstance().runAmpForwards(),
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that runs the amp subsystem backwards.
     * 
     * @return The command that runs the amp subsystem backwards.
     */
    public static Command getRunAmpBackwardsCommand() {
        return new RunCommand(
            () -> Amp.getInstance().runAmpBackwards(),
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that stops the amp subsystem.
     * 
     * @return The command that stops the amp subsystem.
     */
    public static Command getStopAmpCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().stopAmp(),
            Amp.getInstance()
        );
    }

    /**
     * Gets the command that sets the amp subsystem position
     * to the receive position. Delay added to allow the amp
     * subsystem position to be updated after the amp
     * subsystem stops moving in the event that the command
     * is canceled abruptly.
     * 
     * @return The command that sets the amp subsystem position
     * to the receive position.
     */
    public static Command getAmpReceiveCommand() {
        return new FunctionalCommand(
            () -> Amp.getInstance().setAmpMovementDirection(AMP_MOVEMENT_DIRECTION.TOWARDS_RECEIVE_POSITION), 
            () -> Amp.getInstance().runAmpForwards(), 
            (isFinished) -> Amp.getInstance().stopAmp(), 
            () -> Amp.getInstance().isLimitSwitchAtPosition(AMP_POSITION.RECEIVE_POSITION)
                && Amp.getInstance().isLimitSwitchAtMagnet(), 
            Amp.getInstance()
        ).andThen(new WaitCommand(AmpConstants.AMP_POSITION_DELAY));
    }

    /**
     * Gets the command that sets the amp subsystem position
     * to the drop position. Delay added to allow the amp
     * subsystem position to be updated after the amp
     * subsystem stops moving in the event that the command
     * is canceled abruptly.
     * 
     * @return The command that sets the amp subsystem position
     * to the drop position.
     */
    public static Command getAmpDropCommand() {
        return new FunctionalCommand(
            () -> Amp.getInstance().setAmpMovementDirection(AMP_MOVEMENT_DIRECTION.TOWARDS_DROP_POSITION), 
            () -> Amp.getInstance().runAmpBackwards(), 
            (isFinished) -> Amp.getInstance().stopAmp(), 
            () -> Amp.getInstance().isLimitSwitchAtPosition(AMP_POSITION.DROP_POSITION)
                && Amp.getInstance().isLimitSwitchAtMagnet(), 
            Amp.getInstance()
        ).andThen(new WaitCommand(AmpConstants.AMP_POSITION_DELAY));
    }
}
