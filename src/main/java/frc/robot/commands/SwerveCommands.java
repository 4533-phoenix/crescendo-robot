package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public final class SwerveCommands {
    public static Command getDefaultDriveCommand() {
        return new InstantCommand(
            () -> Swerve.getInstance().drive(),
            Swerve.getInstance()
        );
    }
}
