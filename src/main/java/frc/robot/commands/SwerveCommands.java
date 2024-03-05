package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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

    public static Command getTrackNoteCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                double deltaX = Swerve.getInstance().getNoteDetector().getNoteX();

                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds(
                            0.0,
                            0.0,
                            -Math.signum(deltaX)
                                * SwerveConstants.TRACK_NOTE_ROTATIONAL_VELOCITY
                        )
                    )
                );
            },
            (isFinished) -> {
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()
                    )
                );
            },
            () -> Math.abs(Swerve.getInstance().getNoteDetector().getNoteX()) <= SwerveConstants.TRACK_NOTE_OFFSET_DEADBAND,
            Swerve.getInstance()
        );
    }

    public static Command getAcquireNoteCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                Intake.getInstance().runIntakeForwards();
                Shooter.getInstance().runLiftForwards();

                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                            SwerveConstants.ACQUIRE_NOTE_LINEAR_VELOCITY, 
                            0.0, 
                            0.0, 
                            Swerve.getInstance().getRobotPose().getRotation()
                        )
                    )
                );                
            }, 
            (isFinished) -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();

                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()
                    )
                );
            }, 
            () -> Shooter.getInstance().isLimitSwitchPressed(), 
            Intake.getInstance(),
            Shooter.getInstance(),
            Swerve.getInstance()
        ).withTimeout(SwerveConstants.ACQUIRE_NOTE_TIMEOUT);
    }

    public static Command getTrackAndAcquireNoteCommand() {
        /*
         * Returns the sequential command group. Note height is
         * divided by 3.0 because if notes are vertical then
         * they make the note seem closer than they are by
         * increasing their dimensions, as notes' dimensions
         * should correlate to how they look on the ground.
         */
        return new SequentialCommandGroup(
            getTrackNoteCommand(),
            getAcquireNoteCommand()
        ).onlyIf(
            () -> 
                Swerve.getInstance().getNoteDetector().canSeeNote()
                && (Swerve.getInstance().getNoteDetector().getNoteHeight() / 3.0)
                    * Swerve.getInstance().getNoteDetector().getNoteWidth()
                    >= SwerveConstants.NOTE_DIMENSIONS_DEADBAND
        );
    }

    public static Command getSetSlowModeCommand(boolean isTrue) {
        return new InstantCommand(
            () -> Swerve.getInstance().setSlowMode(isTrue),
            Swerve.getInstance()
        );
    }

    public static Command getToggleRobotRelativeModeCommand() {
        return new InstantCommand(
            () -> Swerve.getInstance().toggleRobotRelativeMode(),
            Swerve.getInstance()
        );
    }
}
