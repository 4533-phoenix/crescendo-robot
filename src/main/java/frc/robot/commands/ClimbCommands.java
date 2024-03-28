package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LeftClimb;
import frc.robot.subsystems.RightClimb;

/**
 * The class for the climb commands.
 */
public class ClimbCommands {
    /**
     * Gets the run left climb up command, which runs the 
     * left climb up. The left climb stops running if 
     * the left climb limit switch reaches the magnet at the 
     * up position. Delay added to allow the left climb
     * subsystem position to be updated after the 
     * left climb subsystem stops moving in the event 
     * that the command is canceled abruptly.
     * 
     * @return The run left climb up command.
     */
    public static Command getRunLeftClimbUpCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> LeftClimb.getInstance().runLeftClimbUp(), 
            (isFinished) -> LeftClimb.getInstance().stopLeftClimb(), 
            () -> LeftClimb.getInstance().isLeftClimbLimitSwitchAtMagnet(), 
            LeftClimb.getInstance());
    }

    /**
     * Gets the run right climb up command, which runs the 
     * right climb up. The right climb stops running if 
     * the right climb limit switch reaches the magnet at the 
     * up position. Delay added to allow the right climb
     * subsystem position to be updated after the 
     * right climb subsystem stops moving in the event 
     * that the command is canceled abruptly.
     * 
     * @return The run right climb up command.
     */
    public static Command getRunRightClimbUpCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> RightClimb.getInstance().runRightClimbUp(), 
            (isFinished) -> RightClimb.getInstance().stopRightClimb(), 
            () -> RightClimb.getInstance().isRightClimbLimitSwitchAtMagnet(),
            RightClimb.getInstance());
    }

    /**
     * Gets the run climb up command, which runs
     * both the left climb and right climb up.
     * 
     * @return The run climb up command.
     */
    public static Command getRunClimbUpCommand() {
        return new ParallelCommandGroup(
            getRunLeftClimbUpCommand(),
            getRunRightClimbUpCommand());
    }
    
    /**
     * Gets the run climb down command, which 
     * runs the climb subsystem down.
     * 
     * @return The run climb down command.
     */
    public static Command getRunClimbDownCommand() {
        return new InstantCommand(
            () -> { 
                LeftClimb.getInstance().runLeftClimbDown();
                RightClimb.getInstance().runRightClimbDown();
            },
            LeftClimb.getInstance(),
            RightClimb.getInstance());
    }

    /**
     * Gets stop climb command, which stops the climb subsystem.
     * 
     * @return The stop climb command.
     */
    public static Command getStopClimbCommand() {
        return new InstantCommand(
            () -> {
                LeftClimb.getInstance().stopLeftClimb();
                RightClimb.getInstance().stopRightClimb();
            },
            LeftClimb.getInstance(),
            RightClimb.getInstance());
    }
}