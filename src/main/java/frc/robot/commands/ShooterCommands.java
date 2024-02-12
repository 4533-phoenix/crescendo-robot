package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    public static Command getRunShooterForwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runShooterForwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getRunShooterBackwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runShooterBackwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getStopShooterCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopShooter(),
            Shooter.getInstance()
        );
    }
}
