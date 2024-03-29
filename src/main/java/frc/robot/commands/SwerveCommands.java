package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.helpers.LimelightHelper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * The class for the swerve commands.
 */
public final class SwerveCommands {
    /**
     * The current previous note detector delta x.
     */
    private static double prevNoteDetectorDeltaX = 0.0;

    /**
     * The current previous note detector latency.
     */
    private static double prevNoteDetectorLatency = 0.0;

    /**
     * The current expected note detector delta x.
     * This is the current note detector delta x that
     * the note detector is expected to be at
     * after adjusting for latency.
     */
    private static double expectedNoteDetectorDeltaX = 0.0;

    /**
     * Sets the current previous note detector delta x.
     * 
     * @param prevNoteDetectorDeltaX The current previous
     * note detector delta x.
     */
    public static void setPrevNoteDetectorDeltaX(double prevNoteDetectorDeltaX) {
        SwerveCommands.prevNoteDetectorDeltaX = prevNoteDetectorDeltaX;
    }

    /**
     * Sets the current previous note detector latency.
     * 
     * @param prevNoteDetectorLatency The current previous
     * note detector latency.
     */
    public static void setPrevNoteDetectorLatency(double prevNoteDetectorLatency) {
        SwerveCommands.prevNoteDetectorLatency = prevNoteDetectorLatency;
    }

    /**
     * Sets the current expected note detector delta x.
     * 
     * @param expectedNoteDetectorDeltaX The current
     * expected note detector delta x.
     */
    public static void setExpectedNoteDetectorDeltaX(double expectedNoteDetectorDeltaX) {
        SwerveCommands.expectedNoteDetectorDeltaX = expectedNoteDetectorDeltaX;
    }

    /**
     * Gets the current previous note detector delta x.
     * 
     * @return The current previous note detector delta x.
     */
    public static double getPrevNoteDetectorDeltaX() {
        return SwerveCommands.prevNoteDetectorDeltaX;
    }

    /**
     * Gets the current previous note detector latency.
     * 
     * @return The current previous note detector latency.
     */
    public static double getPrevNoteDetectorLatency() {
        return SwerveCommands.prevNoteDetectorLatency;
    }

    /**
     * Gets the current expected note detector delta x.
     * 
     * @return The current expected note detector delta x.
     */
    public static double getExpectedNoteDetectorDeltaX() {
        return SwerveCommands.expectedNoteDetectorDeltaX;
    }

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
    public static Command getDefaultDriveCommand(
            DoubleSupplier getX, 
            DoubleSupplier getY, 
            DoubleSupplier getRotation) {
        return new InstantCommand(
            () -> Swerve.getInstance().drive(
                getX.getAsDouble(), 
                getY.getAsDouble(), 
                getRotation.getAsDouble()
            ),
            Swerve.getInstance());
    }

    /**
     * Gets the stop swerve command, which
     * sets the swerve drive subsystem
     * to stop driving.
     * 
     * @return The stop swerve command.
     */
    public static Command getStopSwerveCommand() {
        return new InstantCommand(
            () -> {
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()));
            },
            Swerve.getInstance());
    }

    /**
     * Gets the track note command, which
     * rotates the robot to point at a
     * note in front of it. The command
     * will end when it has successfully
     * pointed the robot at a note or if
     * it can no longer see a note.
     * 
     * @return The track note command.
     */
    public static Command getTrackNoteCommand() {
        return new FunctionalCommand(
            () -> {
                // Get the note detector delta x.
                double deltaX = 
                    LimelightHelper.getTX(LimelightConstants.LIMELIGHT_NAME);

                // Get the note detector latency.
                double latency =
                    LimelightHelper.getLatency_Capture(
                        LimelightConstants.LIMELIGHT_NAME)
                    + LimelightHelper.getLatency_Pipeline(
                        LimelightConstants.LIMELIGHT_NAME);

                /*
                 * Set the current previous note detector delta x
                 * to the current note detector delta x.
                 */
                setPrevNoteDetectorDeltaX(deltaX);

                /*
                 * Set the current previous note detector latency
                 * to the current note detector latency.
                 */
                setPrevNoteDetectorLatency(latency);

                /*
                 * Set the current expected note detector delta x
                 * to the current note detector delta x.
                 */
                setExpectedNoteDetectorDeltaX(deltaX);
            },
            () -> {
                // Get the note detector delta x.
                double deltaX = 
                    LimelightHelper.getTX(LimelightConstants.LIMELIGHT_NAME);

                // Get the note detector latency.
                double latency =
                    LimelightHelper.getLatency_Capture(
                        LimelightConstants.LIMELIGHT_NAME)
                    + LimelightHelper.getLatency_Pipeline(
                        LimelightConstants.LIMELIGHT_NAME);

                // Create the expected delta x.
                double expectedDeltaX = 0.0;
                
                /*
                 * If the current previous note detector latency
                 * equals zero, then set the expected delta x 
                 * to the current note detector delta x, and if not, 
                 * then calculate the expected delta x normally.
                 * This check is done in order to prevent a
                 * scenario wherein calculating the expected
                 * delta x the current previous note detector
                 * latency equals zero and is divided by in
                 * calculating the delta x rate of change.
                 */
                if (getPrevNoteDetectorLatency() == 0) {
                    expectedDeltaX = deltaX;
                } else {
                    /*
                     * Get the delta x rate of change, which
                     * is approximately how fast the note
                     * detector delta x is changing at
                     * the current moment of time. Uses
                     * the current previous note detector
                     * latency due to that being the
                     * difference in time between
                     * the current note detector delta x
                     * and the current previous note detector
                     * delta x.
                     */
                    double deltaXRateOfChange = 
                        (deltaX - getPrevNoteDetectorDeltaX())
                            / getPrevNoteDetectorLatency();
                    
                    /*
                     * Get the expected delta x, which multiplies
                     * the delta x rate of change by the current
                     * latency in order to get the approximate 
                     * change in delta x from the current note detector
                     * delta x to the actual delta x, which is
                     * then added to the current note detector
                     * delta x to get the expected delta x.
                     */
                    expectedDeltaX = deltaX + (deltaXRateOfChange * latency);
                }

                /*
                 * Set the current previous note detector delta x
                 * to the current note detector delta x.
                 */
                setPrevNoteDetectorDeltaX(deltaX);

                /*
                 * Set the current previous note detector latency
                 * to the current note detector latency.
                 */
                setPrevNoteDetectorLatency(latency);

                /*
                 * Set the current expected note detector delta x
                 * to the calculated expected note detector delta x.
                 */
                setExpectedNoteDetectorDeltaX(expectedDeltaX);

                /*
                 * Set the swerve drive subsystem to rotate
                 * towards the note. The rotational velocity
                 * to set the swerve drive subsystem to
                 * is made to scale quadratically with the 
                 * expected delta x and is also clamped to a 
                 * rotational velocity range. This calculated 
                 * rotational velocity is then set to rotate 
                 * the robot towards the note, which is done by 
                 * multiplying it by the opposite sign of the 
                 * sign of the expected delta x.
                 */
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds(
                            0.0,
                            0.0,
                            -Math.signum(expectedDeltaX)
                                * MathUtil.clamp(
                                    Math.pow(expectedDeltaX, 2.0)
                                        * SwerveConstants.TRACK_NOTE_ROTATIONAL_VELOCITY,
                                    SwerveConstants.TRACK_NOTE_SLOW_ROTATIONAL_VELOCITY,
                                    SwerveConstants.TRACK_NOTE_ROTATIONAL_VELOCITY))));
            },
            (isFinished) -> {
                /*
                 * Set the swerve drive subsystem to stop after the robot
                 * has rotated to point towards the note.
                 */
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()));
            },
            () -> Math.abs(getExpectedNoteDetectorDeltaX()) 
                    <= SwerveConstants.TRACK_NOTE_OFFSET_DEADBAND
                || !LimelightHelper.getTV(LimelightConstants.LIMELIGHT_NAME),
            Swerve.getInstance());
    }

    /**
     * Gets the acquire note command, which drives
     * forward with the intake running in order
     * to intake a note in front of the robot.
     * This command will stop after a certain
     * period of time.
     * 
     * @return The acquire note command.
     */
    public static Command getAcquireNoteCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                // Run the intake forwards and the lift forwards.
                Intake.getInstance().runIntakeForwards();
                Shooter.getInstance().runLiftForwards();

                /*
                 * Set the swerve drive subsystem to drive forward
                 * at the acquire note linear velocity.
                 */
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds(
                            SwerveConstants.ACQUIRE_NOTE_LINEAR_VELOCITY, 
                            0.0, 
                            0.0)));                
            }, 
            (isFinished) -> {
                /*
                 * Stop the intake and the lift after the note
                 * has been acquired.
                 */
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();

                /*
                 * Set the swerve drive subsystem to stop after
                 * the note has been acquired.
                 */
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()));
            }, 
            () -> Shooter.getInstance().isLimitSwitchPressed(),
            Intake.getInstance(),
            Shooter.getInstance(),
            Swerve.getInstance())
                .withTimeout(SwerveConstants.ACQUIRE_NOTE_TIMEOUT);
    }

    /**
     * Gets the track and acquire note command, which
     * tracks a note in front of the robot and
     * acquires it. This command will only register
     * if the note detector can see a note in front
     * of the robot and if the note is within a
     * certain distance of the robot, which is done
     * by checking the area that the note takes up
     * in the camera frame and comparing it to
     * the note dimensions deadband.
     * 
     * @return The track and acquire note command.
     */
    public static Command getTrackAndAcquireNoteCommand() {
        return new SequentialCommandGroup(
            getTrackNoteCommand(),
            getAcquireNoteCommand())
                .onlyIf(
                    () -> 
                        LimelightHelper.getTV(LimelightConstants.LIMELIGHT_NAME)
                            && LimelightHelper.getTA(LimelightConstants.LIMELIGHT_NAME)
                                >= SwerveConstants.NOTE_DIMENSIONS_DEADBAND);
    }

    /**
     * Gets the stop track and acquire note command,
     * which stops the intake and lift from running
     * and stops the swerve drive subsystem. Used
     * if the track and acquire note button is released
     * before the track and acquire note command is
     * finished.
     * 
     * @return The stop track and acquire note command.
     */
    public static Command getStopTrackAndAcquireNoteCommand() {
        return new InstantCommand(
            () -> {
                Intake.getInstance().stopIntake();
                Shooter.getInstance().stopLift();

                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()));
            },
            Intake.getInstance(),
            Shooter.getInstance(),
            Swerve.getInstance());
    }

    /**
     * Gets the lock wheels command, which locks the
     * wheels to diagonal angles so that the robot will
     * stop any previous movement fast and will be
     * hard to push around. The front swerve modules
     * point diagonally forwards and towards each other
     * and the back swerve modules point diagonally
     * backwards and towards each other, which is done
     * so that if the drive motors on the swerve modules
     * start running then the robot should not move much.
     * 
     * @return The lock wheels command.
     */
    public static Command getLockWheelsCommand() {
        return new RunCommand(
            () -> {
                Swerve.getInstance().setSwerveModuleAngles(
                    new double[]{
                        (7.0 * Math.PI) / 4.0,
                        Math.PI / 4.0,
                        (5.0 * Math.PI) / 4.0,
                        (3.0 * Math.PI) / 4.0});
            },
            Swerve.getInstance());
    }

    /**
     * Gets the set slow mode command, which
     * sets whether or not swerve drive slow
     * mode is active to the given boolean
     * value.
     * 
     * @param isActive Whether or not swerve
     * drive slow mode will be active.
     * 
     * @return The set slow mode command.
     */
    public static Command getSetSlowModeCommand(boolean isActive) {
        return new InstantCommand(
            () -> Swerve.getInstance().setSlowMode(isActive),
            Swerve.getInstance());
    }

    /**
     * Gets the toggle robot relative mode command,
     * which toggles swerve drive robot relative mode.
     * 
     * @return The toggle robot relative mode command.
     */
    public static Command getToggleRobotRelativeModeCommand() {
        return new InstantCommand(
            () -> Swerve.getInstance().toggleRobotRelativeMode(),
            Swerve.getInstance());
    }
}
