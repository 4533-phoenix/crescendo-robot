package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.LeftClimb;
import frc.robot.subsystems.RightClimb;

/**
 * The class for the climb commands.
 */
public class ClimbCommands {
    /**
     * Gets the command that runs the left climb up.
     * The left climb stops running if the left climb
     * limit switch reaches the magnet at the up
     * position. Delay added to allow the left climb
     * subsystem position to be updated after the 
     * left climb subsystem stops moving in the event 
     * that the command is canceled abruptly.
     * 
     * @return The command that runs the left climb up.
     */
    public static Command getRunLeftClimbUpCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> LeftClimb.getInstance().runLeftClimbUp(), 
            (isFinished) -> LeftClimb.getInstance().stopLeftClimb(), 
            () -> LeftClimb.getInstance().isLeftClimbLimitSwitchAtMagnet(), 
            LeftClimb.getInstance()
        );
    }

    /**
     * Gets the command that runs the right climb up.
     * The right climb stops running if the right climb
     * limit switch reaches the magnet at the up
     * position. Delay added to allow the right climb
     * subsystem position to be updated after the 
     * right climb subsystem stops moving in the event 
     * that the command is canceled abruptly.
     * 
     * @return The command that runs the right climb up.
     */
    public static Command getRunRightClimbUpCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> RightClimb.getInstance().runRightClimbUp(), 
            (isFinished) -> RightClimb.getInstance().stopRightClimb(), 
            () -> RightClimb.getInstance().isRightClimbLimitSwitchAtMagnet(), 
            RightClimb.getInstance()
        );
    }

    public static Command getRunClimbUpCommand() {
        return new ParallelCommandGroup(
            getRunLeftClimbUpCommand(),
            getRunRightClimbUpCommand()
        );
    }

    /**
     * Gets the command that runs the left climb up.
     * The left climb stops running if the left climb
     * limit switch reaches the magnet at the down
     * position. Delay added to allow the left climb
     * subsystem position to be updated after the 
     * left climb subsystem stops moving in the event 
     * that the command is canceled abruptly.
     * 
     * @return The command that runs the left climb up.
     */
    public static Command getRunLeftClimbDownCommand() {
        return new RunCommand(
            () -> LeftClimb.getInstance().runLeftClimbDown(),
            LeftClimb.getInstance()
        );
    }

    /**
     * Gets the command that runs the right climb up.
     * The right climb stops running if the right climb
     * limit switch reaches the magnet at the down
     * position. Delay added to allow the right climb
     * subsystem position to be updated after the 
     * right climb subsystem stops moving in the event 
     * that the command is canceled abruptly.
     * 
     * @return The command that runs the right climb up.
     */
    public static Command getRunRightClimbDownCommand() {
        return new RunCommand(
            () -> RightClimb.getInstance().runRightClimbDown(),
            RightClimb.getInstance()
        );
    }
    
    /**
     * Gets the command that runs the climb subsystem down.
     * 
     * @return The command that runs the climb subsystem down.
     */
    public static Command getRunClimbDownCommand() {
        return new ParallelCommandGroup(
            getRunLeftClimbDownCommand(),
            getRunRightClimbDownCommand()
        );
    }

    /**
     * Gets the command that stops the climb subsystem.
     * 
     * @return The command that stops the climb subsystem.
     */
    public static Command getStopClimbCommand() {
        return new InstantCommand(
            () -> {
                LeftClimb.getInstance().stopLeftClimb();
                RightClimb.getInstance().stopRightClimb();
            },
            LeftClimb.getInstance(),
            RightClimb.getInstance()
        );
    }
}