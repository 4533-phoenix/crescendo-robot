package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * The class for the auto commands.
 */
public final class AutoCommands {
    /**
     * The current autonomous path planner trajectory.
     */
    private static PathPlannerTrajectory trajectory = null;

    /**
     * The current autonomous path planner event commands.
     */
    private static List<Pair<Double, Command>> eventCommands = null;

    /**
     * The current autonomous time.
     */
    private static double currTime = 0.0;

    /**
     * The current autonomous end time.
     */
    private static double endTime = 0.0;

    /**
     * Sets the current autonomous path planner trajectory 
     * to the given path planner trajectory.
     * 
     * @param trajectory The path planner trajectory
     * to set the current autonomous path planner
     * trajectory to.
     */
    public static void setTrajectory(PathPlannerTrajectory trajectory) {
        AutoCommands.trajectory = trajectory;
    }

    /**
     * Sets the current autonomous event commands
     * to the given event commands.
     * 
     * @param eventCommands The event commands
     * to set the current autonomous event
     * commands to.
     */
    public static void setEventCommands(List<Pair<Double, Command>> eventCommands) {
        AutoCommands.eventCommands = eventCommands;
    }

    /**
     * Sets the current autonomous time to the
     * given time.
     * 
     * @param currTime The time to set the current
     * autonomous time to.
     */
    public static void setCurrTime(double currTime) {
        AutoCommands.currTime = currTime;
    }

    /**
     * Sets the current autonomous end time to
     * the given time.
     * 
     * @param endTime The time to set the current
     * autonomous end time to.
     */
    public static void setEndTime(double endTime) {
        AutoCommands.endTime = endTime;
    }

    /**
     * Gets the current autonomous path planner trajectory.
     * 
     * @return The current autonomous path planner trajectory.
     */
    public static PathPlannerTrajectory getTrajectory() {
        return AutoCommands.trajectory;
    }

    /**
     * Gets the current autonomous path planner event commands.
     * 
     * @return The current autonomous path planner event commands.
     */
    public static List<Pair<Double, Command>> getEventCommands() {
        return AutoCommands.eventCommands;
    }

    /**
     * Gets the current autonomous time.
     * 
     * @return The current autonomous time.
     */
    public static double getCurrTime() {
        return AutoCommands.currTime;
    }

    /**
     * Gets the current autonomous end time.
     * 
     * @return The current autonomous end time.
     */
    public static double getEndTime() {
        return AutoCommands.endTime;
    }

    /**
     * Gets an auto command that follows the path corresponding to the given
     * path file name.
     * 
     * @param pathFile The path file name.
     * 
     * @return An auto command that follows the path corresponding to the given
     * path file name.
     */
    public static Command followPathAuto(String pathFile) {
        return new FunctionalCommand(
            () -> {
                // Get the current alliance from driver station.
                Optional<Alliance> driverStationAlliance = DriverStation.getAlliance();

                // Create the alliance.
                Alliance alliance = null;
                
                /*
                 * If the current alliance from driver station is present, 
                 * then set the alliance to the current alliance from driver 
                 * station, and if not, then set it to the blue alliance.
                 */
                alliance = driverStationAlliance.isPresent() 
                    ? driverStationAlliance.get()
                    : Alliance.Blue;

                // Create the path.
                PathPlannerPath path = null;

                /*
                 * If the path file exists, then set the path,
                 * to the path from the path file, and if not,
                 * then print the stack trace of the error and
                 * throw the error.
                 */
                try {
                    path = PathPlannerPath.fromPathFile(pathFile);
                } catch (Exception e) {
                    e.printStackTrace();

                    throw e;
                }

                /*
                 * If the alliance is the red alliance, 
                 * then flip the path, and if not,
                 * then keep the path as is.
                 */
                path = alliance == Alliance.Red
                    ? path.flipPath()
                    : path;

                /*
                 * Reset the swerve drive pose estimator to be at the
                 * first point on the path.
                 */
                Swerve.getInstance().resetPoseEstimator(path.getPreviewStartingHolonomicPose());

                // Set the trajectory from the path.
                AutoCommands.setTrajectory(
                    path.getTrajectory(
                        Swerve.getInstance().getChassisSpeeds(), 
                        Swerve.getInstance().getRobotPose().getRotation())); 

                // Get the event commands for the trajectory.
                AutoCommands.setEventCommands(
                    AutoCommands.getTrajectory().getEventCommands());

                AutoCommands.setCurrTime(Timer.getFPGATimestamp());

                AutoCommands.setEndTime(
                    AutoCommands.getCurrTime() 
                    + AutoCommands.getTrajectory().getTotalTimeSeconds());
            },
            () -> {
                PathPlannerTrajectory trajectory = AutoCommands.getTrajectory();
                List<Pair<Double, Command>> eventCommands = AutoCommands.getEventCommands();
                double currTime = AutoCommands.getCurrTime();
                double endTime = AutoCommands.getEndTime();

                State trajectoryState = trajectory.sample(
                    trajectory.getTotalTimeSeconds() - (endTime - currTime));

                // Get the trajectory state velocity and translation.
                double trajectoryStateDesiredVelocity = trajectoryState.velocityMps;
                Translation2d trajectoryStateTranslation = trajectoryState.positionMeters;

                /*
                 * Get the trajectory state rotation and heading.
                 * 
                 * In PathPlannerLib, heading is the rotation of the
                 * state, while target holonomic rotation is the 
                 * desired heading.
                 */
                Rotation2d trajectoryStateRotation = trajectoryState.heading;
                Rotation2d trajectoryStateDesiredHeading = trajectoryState.targetHolonomicRotation;

                // Get the chassis speeds as calculated by the swerve subsystem holonomic drive controller.
                ChassisSpeeds chassisSpeeds = Swerve.getInstance().getHolonomicDriveController().calculate(
                    Swerve.getInstance().getRobotPose(), 
                    new Pose2d(trajectoryStateTranslation, trajectoryStateRotation), 
                    trajectoryStateDesiredVelocity, 
                    trajectoryStateDesiredHeading
                );

                /*
                 * Get the swerve module states as calculated by the swerve drive kinematics 
                 * given the chassis speeds.
                 */
                SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                // Set the swerve subsystem to drive at the swerve module states.
                Swerve.getInstance().setSwerveModuleStates(swerveModuleStates);
                   
                /*
                 * If the frontmost event command is triggered, then schedule it
                 * with the command scheduler and remove it from the front
                 * of the event commands list.
                 */
                if (eventCommands.size() != 0 && eventCommands.get(0).getFirst() <= trajectoryState.timeSeconds) {
                    // CommandScheduler.getInstance().schedule(eventCommands.remove(0).getSecond());
                }

                // Update the current time.
                AutoCommands.setCurrTime(Timer.getFPGATimestamp());
            },
            (isFinished) -> {
                // Set the swerve subsystem to stop after the path has finished.
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()));
            },
            () -> AutoCommands.getCurrTime() > AutoCommands.getEndTime(),
            Swerve.getInstance()
        );
    }

    public static Command getSpeakerScoreAuto() {
        return new SequentialCommandGroup(
            ShooterCommands.getShootNoteCommand(),
            followPathAuto(AutoConstants.SPEAKER_SCORE_AUTO_PATH_FILE_NAME)
        );
    }

    public static Command getDoubleSpeakerScoreAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    // Get the current alliance from driver station.
                    Optional<Alliance> driverStationAlliance = DriverStation.getAlliance();

                    // Create the alliance.
                    Alliance alliance = null;
                    
                    /*
                    * If the current alliance from driver station is present, 
                    * then set the alliance to the current alliance from driver 
                    * station, and if not, then set it to the blue alliance.
                    */
                    alliance = driverStationAlliance.isPresent() 
                        ? driverStationAlliance.get()
                        : Alliance.Blue;

                    Pose2d[] pointsOfInterest = alliance == Alliance.Red
                        ? flipPoses(AutoConstants.POINTS_OF_INTEREST)
                        : AutoConstants.POINTS_OF_INTEREST;

                    Swerve.getInstance().resetPoseEstimator(pointsOfInterest[2]);
                },
                Swerve.getInstance()
            ),
            ShooterCommands.getShootNoteCommand(),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPointOfInterestCommand(),
            ShooterCommands.getShootNoteCommand()
            // followPathAuto("Top Note Score Path"),
            // SwerveCommands.getTrackAndAcquireNoteCommand(),
            // getDriveToPointOfInterestCommand(),
            // ShooterCommands.getShootNoteCommand()
        );
    }

    public static Command getDriveToPointOfInterestCommand() {
        return new FunctionalCommand(
            () -> {
                // Get the current alliance from driver station.
                Optional<Alliance> driverStationAlliance = DriverStation.getAlliance();

                // Create the alliance.
                Alliance alliance = null;
                
                /*
                 * If the current alliance from driver station is present, 
                 * then set the alliance to the current alliance from driver 
                 * station, and if not, then set it to the blue alliance.
                 */
                alliance = driverStationAlliance.isPresent() 
                    ? driverStationAlliance.get()
                    : Alliance.Blue;

                Pose2d robotPose = Swerve.getInstance().getRobotPose();

                Pose2d[] pointsOfInterest = alliance == Alliance.Red
                    ? flipPoses(AutoConstants.POINTS_OF_INTEREST)
                    : AutoConstants.POINTS_OF_INTEREST;

                pointsOfInterest = Shooter.getInstance().isLimitSwitchPressed()
                    ? new Pose2d[]{
                        pointsOfInterest[2],
                        pointsOfInterest[3],
                        pointsOfInterest[4]}
                    : new Pose2d[]{pointsOfInterest[0], pointsOfInterest[1]};
                
                Pose2d pointOfInterest = 
                    robotPose.nearest(Arrays.asList(pointsOfInterest));

                PathPlannerPath path = null;

                if (robotPose.getTranslation().getDistance(pointOfInterest.getTranslation())
                        > AutoConstants.POINT_OF_INTEREST_DISTANCE_DEADBAND) {
                    path = new PathPlannerPath(
                        Arrays.asList(
                            new Translation2d[]{
                                robotPose.getTranslation(),
                                robotPose.getTranslation(),
                                robotPose.getTranslation(),
                                robotPose.getTranslation()}), 
                        new PathConstraints(
                            0.0, 
                            0.0, 
                            0.0, 
                            0.0), 
                        new GoalEndState(
                            0.0, 
                            new Rotation2d()));
                } else {
                    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                        robotPose,
                        pointOfInterest
                    );

                    path = new PathPlannerPath(
                        bezierPoints, 
                        new PathConstraints(
                            AutoConstants.MAX_VELOCITY, 
                            AutoConstants.MAX_ACCELERATION, 
                            AutoConstants.MAX_ROTATIONAL_VELOCITY, 
                            AutoConstants.MAX_ROTATIONAL_ACCELERATION),
                        new GoalEndState(
                            0.0, 
                            pointOfInterest.getRotation(), 
                            true));
                }
                
                
                // Set the trajectory from the path.
                AutoCommands.setTrajectory(
                    path.getTrajectory(
                        Swerve.getInstance().getChassisSpeeds(), 
                        robotPose.getRotation())); 

                // Get the event commands for the trajectory.
                AutoCommands.setEventCommands(
                    AutoCommands.getTrajectory().getEventCommands());

                AutoCommands.setCurrTime(Timer.getFPGATimestamp());

                AutoCommands.setEndTime(
                    AutoCommands.getCurrTime() 
                    + AutoCommands.getTrajectory().getTotalTimeSeconds());
            }, 
            () -> {
                PathPlannerTrajectory trajectory = AutoCommands.getTrajectory();
                List<Pair<Double, Command>> eventCommands = AutoCommands.getEventCommands();
                double currTime = AutoCommands.getCurrTime();
                double endTime = AutoCommands.getEndTime();

                State trajectoryState = trajectory.sample(
                    trajectory.getTotalTimeSeconds() - (endTime - currTime));

                // Get the trajectory state velocity and translation.
                double trajectoryStateDesiredVelocity = trajectoryState.velocityMps;
                Translation2d trajectoryStateTranslation = trajectoryState.positionMeters;

                /*
                 * Get the trajectory state rotation and heading.
                 * 
                 * In PathPlannerLib, heading is the rotation of the
                 * state, while target holonomic rotation is the 
                 * desired heading.
                 */
                Rotation2d trajectoryStateRotation = trajectoryState.heading;
                Rotation2d trajectoryStateDesiredHeading = trajectoryState.targetHolonomicRotation;

                // Get the chassis speeds as calculated by the swerve subsystem holonomic drive controller.
                ChassisSpeeds chassisSpeeds = Swerve.getInstance().getHolonomicDriveController().calculate(
                    Swerve.getInstance().getRobotPose(), 
                    new Pose2d(trajectoryStateTranslation, trajectoryStateRotation), 
                    trajectoryStateDesiredVelocity, 
                    trajectoryStateDesiredHeading
                );

                /*
                 * Get the swerve module states as calculated by the swerve drive kinematics 
                 * given the chassis speeds.
                 */
                SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                // Set the swerve subsystem to drive at the swerve module states.
                Swerve.getInstance().setSwerveModuleStates(swerveModuleStates);
                   
                /*
                 * If the frontmost event command is triggered, then schedule it
                 * with the command scheduler and remove it from the front
                 * of the event commands list.
                 */
                if (eventCommands.size() != 0 && eventCommands.get(0).getFirst() <= trajectoryState.timeSeconds) {
                    // CommandScheduler.getInstance().schedule(eventCommands.remove(0).getSecond());
                }

                // Update the current time.
                AutoCommands.setCurrTime(Timer.getFPGATimestamp());
            },
            (isFinished) -> {
                // Set the swerve subsystem to stop after the path has finished.
                Swerve.getInstance().setSwerveModuleStates(
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds()));
            },
            () -> AutoCommands.getCurrTime() > AutoCommands.getEndTime(),
            Swerve.getInstance());
    }

    public static Pose2d[] flipPoses(Pose2d[] poses) {
        Pose2d[] flippedPoses = new Pose2d[poses.length];

        for (int i = 0; i < poses.length; i++) {
            flippedPoses[i] = new Pose2d(
                AutoConstants.FIELD_LENGTH - poses[i].getX(), 
                poses[i].getY(), 
                new Rotation2d(Math.PI).minus(poses[i].getRotation()));
        }

        return flippedPoses;
    }
}
