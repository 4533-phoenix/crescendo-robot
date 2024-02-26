package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

/**
 * The class for the swerve commands.
 */
public final class SwerveCommands {
    /**
     * Gets the default drive command given functions
     * that return the x, y, and rotation velocity factors 
     * for the drive method.
     * 
     * @param getX The function that returns the x velocity factor.
     * @param getY The function that returns the y velocity factor.
     * @param getRotation The function that returns the rotation velocity factor.
     * 
     * @return The default drive command.
     */
    public static Command getDefaultDriveCommand(DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRotation) {
        return new InstantCommand(
            () -> Swerve.getInstance().drive(
                getX.getAsDouble(), 
                getY.getAsDouble(), 
                getRotation.getAsDouble()
            ),
            Swerve.getInstance()
        );
    }
}
