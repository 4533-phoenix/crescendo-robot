package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    public static RunCommand getRunShooterForwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runShooterForwards(), 
            Shooter.getInstance()
        );
    }

    public static RunCommand getRunShooterBackwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runShooterBackwards(), 
            Shooter.getInstance()
        );
    }

    public static InstantCommand getStopShooterCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopShooter(),
            Shooter.getInstance()
        );
    }
}
