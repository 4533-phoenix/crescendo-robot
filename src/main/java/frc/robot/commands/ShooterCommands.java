package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    public static Command getRunLeftShooterForwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runLeftShooterForwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getRunLeftShooterBackwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runLeftShooterBackwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getRunRightShooterForwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runRightShooterForwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getRunRightShooterBackwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runRightShooterBackwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getStopLeftShooterCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopLeftShooter(),
            Shooter.getInstance()
        );
    }

    public static Command getStopRightShooterCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopRightShooter(),
            Shooter.getInstance()
        );
    }

    public static Command getRunLiftForwardsCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runLiftForwards(),
            Shooter.getInstance()
        );
    }

    public static Command getRunLiftBackwardsCommmand() {
        return new RunCommand(
            () -> Shooter.getInstance().runLiftBackwards(), 
            Shooter.getInstance()
        );
    }

    public static Command getStopLiftCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopLift(),
            Shooter.getInstance()
        );
    }

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

    public static Command getShootNoteCommand() {
        return new SequentialCommandGroup(
            getRunRightShooterForwardsCommand().withTimeout(0.7),
            getRunShooterForwardsCommand().withTimeout(1.3),
            getStopShooterCommand()
        );
    }

    public static Command getIntakeNoteCommand() {
        return new FunctionalCommand(
            () -> Shooter.getInstance().setLeftShooterIdleMode(IdleMode.kBrake),
            () -> {
                Intake.getInstance().runIntakeForwards();
                Shooter.getInstance().runLiftForwards();
            },
            (isFinished) -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();

                Shooter.getInstance().setLeftShooterIdleMode(IdleMode.kCoast);
            },
            () -> Shooter.getInstance().isLimitSwitchPressed(),
            Intake.getInstance(),
            Shooter.getInstance()
        );
    }

    public static Command stopIntakeNoteCommand() {
        return new InstantCommand(
            () -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();
            },
            Intake.getInstance(),
            Shooter.getInstance()
        );
    }
}
