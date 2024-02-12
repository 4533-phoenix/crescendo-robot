package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climb;

public class ClimbCommands {
     public static RunCommand getRunClimbDownwardsCommand() {
        return new RunCommand(
            () -> Climb.getInstance().runClimbDown(),
            Climb.getInstance()
        );
    }

    public static RunCommand getRunClimbUpwardsCommand() {
        return new RunCommand(
            () -> Climb.getInstance().runClimbUp(),
            Climb.getInstance()
        );
    }

    public static InstantCommand getStopClimbCommand() {
        return new InstantCommand(
            () -> Climb.getInstance().stopClimb(),
            Climb.getInstance()
        );
    }
}
