package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.helpers.LimelightHelper;

/**
 * The class for the shooter commands.
 */
public class LimelightCommands {
    /**
     * Gets the run lift forwards command, which runs
     * the lift forwards.
     * 
     * @return The run lift forwards command.
     */
    public static Command getStartFlashLimelightCommand() {
        return new InstantCommand(
            () -> LimelightHelper.setLEDMode_ForceBlink(LimelightConstants.LIMELIGHT_NAME));
    }

    /**
     * Gets the run lift backwards command, which runs
     * the lift backwards.
     * 
     * @return The run lift backwards command.
     */
    public static Command getStopFlashLimelightCommand() {
        return new InstantCommand(
            () -> LimelightHelper.setLEDMode_ForceOff(LimelightConstants.LIMELIGHT_NAME));
    }
}
