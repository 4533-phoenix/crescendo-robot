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
                Swerve.getInstance().resetPoseEstimator(
                    path.getPreviewStartingHolonomicPose());

                /*
                 * Reset the theta controller of the swerve drive
                 * holonomic drive controller to be at the rotation
                 * of the first point on the path.
                 */
                Swerve.getInstance()
                    .getHolonomicDriveController()
                    .getThetaController()
                        .reset(
                            path.getPreviewStartingHolonomicPose()
                                .getRotation()
                                .getRadians());

                // Set the trajectory from the path.
                AutoCommands.setTrajectory(
                    path.getTrajectory(
                        Swerve.getInstance().getChassisSpeeds(), 
                        Swerve.getInstance().getRobotPose().getRotation()));

                // Get the event commands for the trajectory.
                AutoCommands.setEventCommands(
                    AutoCommands.getTrajectory().getEventCommands());

                // Set the current time.
                AutoCommands.setCurrTime(Timer.getFPGATimestamp());

                // Set the end time.
                AutoCommands.setEndTime(
                    AutoCommands.getCurrTime() 
                    + AutoCommands.getTrajectory().getTotalTimeSeconds());
            },
            () -> {
                // Get the trajectory.
                PathPlannerTrajectory trajectory = AutoCommands.getTrajectory();

                // Get the event commands.
                List<Pair<Double, Command>> eventCommands = AutoCommands.getEventCommands();

                // Get the current time.
                double currTime = AutoCommands.getCurrTime();

                // Get the end time.
                double endTime = AutoCommands.getEndTime();

                /*
                 * Get the trajectory state at the current time
                 * along the path.
                 */
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
                Rotation2d trajectoryStateRotation = 
                    trajectoryState.heading;
                
                Rotation2d trajectoryStateDesiredHeading = 
                    trajectoryState.targetHolonomicRotation;

                /*
                 * Get the chassis speeds as calculated by the 
                 * swerve subsystem holonomic drive controller.
                 */
                ChassisSpeeds chassisSpeeds = 
                    Swerve.getInstance()
                        .getHolonomicDriveController()
                        .calculate(
                            Swerve.getInstance().getRobotPose(), 
                            new Pose2d(
                                trajectoryStateTranslation, 
                                trajectoryStateRotation), 
                            trajectoryStateDesiredVelocity, 
                            trajectoryStateDesiredHeading);

                /*
                 * Get the swerve module states as calculated by the 
                 * swerve drive kinematics given the chassis speeds.
                 */
                SwerveModuleState[] swerveModuleStates = 
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS
                        .toSwerveModuleStates(chassisSpeeds);

                // Set the swerve subsystem to drive at the swerve module states.
                Swerve.getInstance().setSwerveModuleStates(swerveModuleStates);
                   
                /*
                 * If the frontmost event command is triggered, then schedule it
                 * with the command scheduler and remove it from the front
                 * of the event commands list.
                 */
                if (eventCommands.size() != 0 
                        && eventCommands.get(0).getFirst() 
                            <= trajectoryState.timeSeconds) {
                    // CommandScheduler.getInstance().schedule(
                    //     eventCommands.remove(0).getSecond());
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

    /**
     * Gets the speaker score auto, which scores
     * one note at subwoofer left on blue
     * alliance or one note at subwoofer right
     * on red alliance, and then leaves the
     * robot starting zone by curving downwards
     * and right.
     * 
     * @return The speaker score auto.
     */
    public static Command getSpeakerScoreAuto() {
        return new SequentialCommandGroup(
            ShooterCommands.getShootNoteCommand(),
            followPathAuto(AutoConstants.SPEAKER_SCORE_AUTO_PATH_FILE_NAME));
    }

    /**
     * Gets the speaker score auto, which scores
     * one note at subwoofer left on blue
     * alliance or one note at subwoofer right
     * on red alliance, then leaves the
     * robot starting zone by curving downwards
     * and right to the bottom center note, and
     * then intakes the bottom center note.
     * 
     * @return The speaker score to center auto.
     */
    public static Command getSpeakerScoreToCenterAuto() {
        return new SequentialCommandGroup(
            ShooterCommands.getShootNoteCommand(),
            followPathAuto(AutoConstants.SPEAKER_SCORE_TO_CENTER_AUTO_PATH_FILE_NAME),
            SwerveCommands.getTrackAndAcquireNoteCommand());
    }

    /**
     * Gets the double speaker score auto, which scores
     * one note at subwoofer center, then intakes the
     * middle alliance note, drives back to subwoofer
     * center, and then scores the middle alliance note.
     * 
     * @return The double speaker score auto.
     */
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

                    /*
                     * If the alliance is the red alliance, then
                     * flip the subwoofer center pose to the red alliance
                     * side, and if not, then keep the subwoofer center
                     * pose the same.
                     */
                    Pose2d initialPose = alliance == Alliance.Red
                        ? flipPose(AutoConstants.SUBWOOFER_CENTER_POSE)
                        : AutoConstants.SUBWOOFER_CENTER_POSE;

                    /*
                     * Reset the pose estimator to the initial pose,
                     * which is the subwoofer center pose.
                     */
                    Swerve.getInstance().resetPoseEstimator(initialPose);
                },
                Swerve.getInstance()
            ),
            ShooterCommands.getShootNoteCommand(),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPointOfInterestCommand(),
            ShooterCommands.getShootNoteCommand());
    }

    /**
     * Gets the triple speaker score auto, which scores
     * one note at subwoofer center, then intakes the
     * middle alliance note, drives back to subwoofer
     * center, scores the middle alliance note, then
     * intakes the top alliance note, drives back to
     * subwoofer center, and scores the top alliance
     * note.
     * 
     * @return The triple speaker score auto.
     */
    public static Command getTripleSpeakerScoreAuto() {
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

                    /*
                     * If the alliance is the red alliance, then
                     * flip the subwoofer center pose to the red alliance
                     * side, and if not, then keep the subwoofer center
                     * pose the same.
                     */
                    Pose2d initialPose = alliance == Alliance.Red
                        ? flipPose(AutoConstants.SUBWOOFER_CENTER_POSE)
                        : AutoConstants.SUBWOOFER_CENTER_POSE;

                    /*
                     * Reset the pose estimator to the initial pose,
                     * which is the subwoofer center pose.
                     */
                    Swerve.getInstance().resetPoseEstimator(initialPose);
                },
                Swerve.getInstance()),
            ShooterCommands.getShootNoteCommand(),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPoseCommand(AutoConstants.SUBWOOFER_CENTER_POSE),
            ShooterCommands.getShootNoteCommand(),
            followPathAuto(AutoConstants.TOP_NOTE_PATH),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPoseCommand(AutoConstants.SUBWOOFER_CENTER_POSE),
            ShooterCommands.getShootNoteCommand());
    }

    /**
     * Gets the triple speaker score auto, which scores
     * one note at subwoofer center, then intakes the
     * middle alliance note, drives back to subwoofer
     * center, scores the middle alliance note, then
     * intakes the top alliance note, drives back to
     * subwoofer center, and scores the top alliance
     * note.
     * 
     * @return The triple speaker score auto.
     */
    public static Command getQuadrupleSpeakerScoreAuto() {
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

                    /*
                     * If the alliance is the red alliance, then
                     * flip the subwoofer center pose to the red alliance
                     * side, and if not, then keep the subwoofer center
                     * pose the same.
                     */
                    Pose2d initialPose = alliance == Alliance.Red
                        ? flipPose(AutoConstants.SUBWOOFER_CENTER_POSE)
                        : AutoConstants.SUBWOOFER_CENTER_POSE;

                    /*
                     * Reset the pose estimator to the initial pose,
                     * which is the subwoofer center pose.
                     */
                    Swerve.getInstance().resetPoseEstimator(initialPose);
                },
                Swerve.getInstance()),
            ShooterCommands.getShootNoteCommand(),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPoseCommand(AutoConstants.SUBWOOFER_CENTER_POSE),
            ShooterCommands.getShootNoteCommand(),
            followPathAuto(AutoConstants.TOP_NOTE_PATH),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPoseCommand(AutoConstants.SUBWOOFER_CENTER_POSE),
            ShooterCommands.getShootNoteCommand(),
            followPathAuto("Bottom Note Path"),
            SwerveCommands.getTrackAndAcquireNoteCommand(),
            getDriveToPoseCommand(AutoConstants.SUBWOOFER_CENTER_POSE),
            ShooterCommands.getShootNoteCommand());
    }

    /**
     * Gets the drive to point of interest command, which
     * will drive to either the amp or source without a
     * note in the shooter and will drive to one of the
     * subwoofer positions with a note in the shooter.
     * The point of interest that will be driven to
     * will be the nearer of the available points of
     * interest to drive to.
     * 
     * @return The drive to point of interest command.
     */
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

                // Get the current robot pose.
                Pose2d robotPose = Swerve.getInstance().getRobotPose();

                /*
                 * If there is a note in the shooter, then set the
                 * points of interest to only be the shooter points
                 * of interest, and if not, then set the points of
                 * interest to only be the default points of interest.
                 */
                Pose2d[] pointsOfInterest = Shooter.getInstance().isLimitSwitchPressed()
                    ? AutoConstants.SHOOTER_POINTS_OF_INTEREST
                    : AutoConstants.DEFAULT_POINTS_OF_INTEREST;

                /*
                 * If the alliance is the red alliance, then
                 * flip the points of interest to the red alliance
                 * side, and if not, then keep the points of interest
                 * the same.
                 */
                pointsOfInterest = alliance == Alliance.Red
                    ? flipPoses(pointsOfInterest)
                    : pointsOfInterest;
                
                /*
                 * Get the point of interest to drive to as
                 * the points of interest nearest to the
                 * current robot pose.
                 */
                Pose2d pointOfInterest = 
                    robotPose.nearest(Arrays.asList(pointsOfInterest));
                
                // Create the path.
                PathPlannerPath path = null;
                
                /*
                 * If the distance from the current robot pose
                 * to the point of interest is greater than the
                 * point of interest distance deadband, then
                 * set the path as a path that drives nowhere,
                 * and if not, then set the path as a path
                 * that drives from the current robot pose
                 * to the point of interest.
                 */
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
                    path = new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                            robotPose,
                            pointOfInterest), 
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

                /*
                 * Reset the theta controller of the swerve drive
                 * holonomic drive controller to be at the rotation
                 * of the current robot pose.
                 */
                Swerve.getInstance()
                    .getHolonomicDriveController()
                    .getThetaController()
                        .reset(robotPose.getRotation().getRadians());
                
                // Set the trajectory from the path.
                AutoCommands.setTrajectory(
                    path.getTrajectory(
                        Swerve.getInstance().getChassisSpeeds(),
                        robotPose.getRotation())); 

                // Get the event commands for the trajectory.
                AutoCommands.setEventCommands(
                    AutoCommands.getTrajectory().getEventCommands());

                // Set the current time.
                AutoCommands.setCurrTime(Timer.getFPGATimestamp());

                // Set the end time.
                AutoCommands.setEndTime(
                    AutoCommands.getCurrTime() 
                    + AutoCommands.getTrajectory().getTotalTimeSeconds());
            }, 
            () -> {
                // Get the trajectory.
                PathPlannerTrajectory trajectory = AutoCommands.getTrajectory();

                // Get the event commands.
                List<Pair<Double, Command>> eventCommands = AutoCommands.getEventCommands();

                // Get the current time.
                double currTime = AutoCommands.getCurrTime();

                // Get the end time.
                double endTime = AutoCommands.getEndTime();
                
                /*
                 * Get the trajectory state at the current time
                 * along the path.
                 */
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
                Rotation2d trajectoryStateRotation = 
                    trajectoryState.heading;

                Rotation2d trajectoryStateDesiredHeading = 
                    trajectoryState.targetHolonomicRotation;

                /*
                 * Get the chassis speeds as calculated by the 
                 * swerve subsystem holonomic drive controller.
                 */
                ChassisSpeeds chassisSpeeds = 
                    Swerve.getInstance().getHolonomicDriveController().calculate(
                        Swerve.getInstance().getRobotPose(), 
                        new Pose2d(trajectoryStateTranslation, trajectoryStateRotation),
                        trajectoryStateDesiredVelocity, 
                        trajectoryStateDesiredHeading);

                /*
                 * Get the swerve module states as calculated by the 
                 * swerve drive kinematics given the chassis speeds.
                 */
                SwerveModuleState[] swerveModuleStates = 
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        chassisSpeeds);

                // Set the swerve subsystem to drive at the swerve module states.
                Swerve.getInstance().setSwerveModuleStates(swerveModuleStates);
                   
                /*
                 * If the frontmost event command is triggered, then schedule it
                 * with the command scheduler and remove it from the front
                 * of the event commands list.
                 */
                if (eventCommands.size() != 0 
                        && eventCommands.get(0).getFirst() 
                            <= trajectoryState.timeSeconds) {
                    // CommandScheduler.getInstance().schedule(
                    //     eventCommands.remove(0).getSecond());
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

    /**
     * Gets the drive to pose command, which will
     * drive the robot to the given pose.
     * 
     * @param pose The pose to drive to. This
     * should be on the blue alliance side,
     * as the command will automatically
     * flip it to the red alliance side
     * if the current alliance is the red
     * alliance.
     */
    public static Command getDriveToPoseCommand(Pose2d pose) {
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

                // Get the current robot pose.
                Pose2d robotPose = Swerve.getInstance().getRobotPose();

                /*
                 * If the alliance is the red alliance, then
                 * flip the pose to drive to, and if not,
                 * then keep it the same.
                 */
                Pose2d endPose = alliance == Alliance.Red
                    ? flipPose(pose)
                    : pose;

                /*
                 * Create the path as a path that drives
                 * from the current robot pose to the
                 * end pose.
                 */
                PathPlannerPath path = new PathPlannerPath(
                    PathPlannerPath.bezierFromPoses(
                        robotPose,
                        endPose), 
                    new PathConstraints(
                        AutoConstants.MAX_VELOCITY, 
                        AutoConstants.MAX_ACCELERATION, 
                        AutoConstants.MAX_ROTATIONAL_VELOCITY, 
                        AutoConstants.MAX_ROTATIONAL_ACCELERATION),
                    new GoalEndState(
                        0.0, 
                        endPose.getRotation(), 
                        true));
                
                /*
                 * Reset the theta controller of the swerve drive
                 * holonomic drive controller to be at the rotation
                 * of the current robot pose.
                 */
                Swerve.getInstance()
                    .getHolonomicDriveController()
                    .getThetaController()
                        .reset(robotPose.getRotation().getRadians());
                
                // Set the trajectory from the path.
                AutoCommands.setTrajectory(
                    path.getTrajectory(
                        Swerve.getInstance().getChassisSpeeds(), 
                        robotPose.getRotation())); 

                // Get the event commands for the trajectory.
                AutoCommands.setEventCommands(
                    AutoCommands.getTrajectory().getEventCommands());

                // Set the current time.
                AutoCommands.setCurrTime(Timer.getFPGATimestamp());

                // Set the end time.
                AutoCommands.setEndTime(
                    AutoCommands.getCurrTime() 
                    + AutoCommands.getTrajectory().getTotalTimeSeconds());
            },
            () -> {
                // Get the trajectory.
                PathPlannerTrajectory trajectory = AutoCommands.getTrajectory();

                // Get the event commands.
                List<Pair<Double, Command>> eventCommands = AutoCommands.getEventCommands();

                // Get the current time.
                double currTime = AutoCommands.getCurrTime();

                // Get the end time.
                double endTime = AutoCommands.getEndTime();
                
                /*
                 * Get the trajectory state at the current time
                 * along the path.
                 */
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
                Rotation2d trajectoryStateRotation = 
                    trajectoryState.heading;
                
                Rotation2d trajectoryStateDesiredHeading = 
                    trajectoryState.targetHolonomicRotation;

                /*
                 * Get the chassis speeds as calculated by the 
                 * swerve subsystem holonomic drive controller.
                 */
                ChassisSpeeds chassisSpeeds = 
                    Swerve.getInstance().getHolonomicDriveController().calculate(
                        Swerve.getInstance().getRobotPose(), 
                        new Pose2d(trajectoryStateTranslation, trajectoryStateRotation), 
                        trajectoryStateDesiredVelocity, 
                        trajectoryStateDesiredHeading);

                /*
                 * Get the swerve module states as calculated by the 
                 * swerve drive kinematics given the chassis speeds.
                 */
                SwerveModuleState[] swerveModuleStates = 
                    SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                        chassisSpeeds);

                // Set the swerve subsystem to drive at the swerve module states.
                Swerve.getInstance().setSwerveModuleStates(swerveModuleStates);
                   
                /*
                 * If the frontmost event command is triggered, then schedule it
                 * with the command scheduler and remove it from the front
                 * of the event commands list.
                 */
                if (eventCommands.size() != 0 
                        && eventCommands.get(0).getFirst() 
                            <= trajectoryState.timeSeconds) {
                    // CommandScheduler.getInstance().schedule(
                    //     eventCommands.remove(0).getSecond());
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

    /**
     * Flips the given poses to the opposite alliance side
     * of the field.
     * 
     * @param poses The poses to flip.
     * 
     * @return The flipped poses.
     */
    public static Pose2d[] flipPoses(Pose2d[] poses) {
        // Create an array to store the flipped poses.
        Pose2d[] flippedPoses = new Pose2d[poses.length];

        /*
         * Loop through the poses and add their
         * flipped versions to the flipped poses
         * array.
         */
        for (int i = 0; i < poses.length; i++) {
            /*
             * Add the flipped pose to the flipped
             * poses array.
             */
            flippedPoses[i] = new Pose2d(
                /*
                 * Flip the x coordinate by subtracting
                 * it from the field length.
                 */
                AutoConstants.FIELD_LENGTH - poses[i].getX(),
                poses[i].getY(),
                // Flip the angle by subtracting it from π.
                new Rotation2d(Math.PI).minus(poses[i].getRotation()));
        }

        return flippedPoses;
    }

    /**
     * Flips the given pose to the opposite alliance side
     * of the field.
     * 
     * @param pose The pose to flip.
     * 
     * @return The flipped pose.
     */
    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(
            /*
             * Flip the x coordinate by subtracting
             * it from the field length.
             */
            AutoConstants.FIELD_LENGTH - pose.getX(),
            pose.getY(),
            // Flip the angle by subtracting it from π.
            new Rotation2d(Math.PI).minus(pose.getRotation()));
    }
}
