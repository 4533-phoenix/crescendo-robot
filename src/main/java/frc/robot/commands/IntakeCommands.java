package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Intake;

public final class IntakeCommands {
    public static RunCommand getRunIntakeForwardsCommand() {
        return new RunCommand(
            () -> Intake.getInstance().runIntakeForward(), 
            Intake.getInstance()
        );
    }

    public static RunCommand getRunIntakeBackwardsCommand() {
        return new RunCommand(
            () -> Intake.getInstance().runIntakeBackward(),
            Intake.getInstance()
        );
    }

    public static InstantCommand getStopIntakeCommand() {
        return new InstantCommand(
            () -> Intake.getInstance().stopIntake(),
            Intake.getInstance()
        );
    }
}
