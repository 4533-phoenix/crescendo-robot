package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public final class SwerveCommands {
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
