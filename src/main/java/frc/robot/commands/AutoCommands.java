package frc.robot.commands;

import java.util.List;
import java.util.Optional;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/**
 * The class for the auto commands.
 */
public final class AutoCommands {
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
        return new InstantCommand(
            () -> {
                /*
                 * Check if the path file exists. If it does not,
                 * then print the stack trace of the error and
                 * return.
                 */
                try {
                    PathPlannerPath.fromPathFile(pathFile);
                } catch (Exception e) {
                    e.printStackTrace();

                    return;
                }

                // Get the current alliance from driver station.
                Optional<Alliance> driverStationAlliance = DriverStation.getAlliance();

                /*
                * If the current alliance from driver station
                * is not present, then return;
                */
                if (!driverStationAlliance.isPresent()) {
                    return;
                }

                Alliance alliance = driverStationAlliance.get();

                /*
                 * Get the path from the path file. If the alliance
                 * is the red alliance, then flip the path, and if not,
                 * then keep the path as is.
                 */
                PathPlannerPath path = alliance == Alliance.Red
                    ? PathPlannerPath.fromPathFile(pathFile).flipPath()
                    : PathPlannerPath.fromPathFile(pathFile);

                /*
                 * Reset the swerve drive pose estimator to be at the
                 * first point on the trajectory.
                 */
                Swerve.getInstance().resetPoseEstimator(path.getPreviewStartingHolonomicPose());

                // Create the trajectory from the path.
                PathPlannerTrajectory trajectory = path.getTrajectory(
                    Swerve.getInstance().getChassisSpeeds(), 
                    Swerve.getInstance().getRobotPose().getRotation()
                );

                // Get the event commands for the trajectory.
                List<Pair<Double, Command>> eventCommands = trajectory.getEventCommands();
                
                // Get the current time.
                double currTime = Timer.getFPGATimestamp();

                // Get the end time of the trajectory.
                double endTime = currTime + trajectory.getTotalTimeSeconds();
                
                // Follow the trajectory while the end time has not been reached.
                while (currTime <= endTime) {
                    // Sample the trajectory at the current elapsed time.
                    State trajectoryState = trajectory.sample(trajectory.getTotalTimeSeconds() - (endTime - currTime));

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

                    if (eventCommands.size() != 0) {
                        System.out.println(eventCommands.get(0).getFirst());
                    }
                    
                    /*
                     * If the frontmost event command is triggered, then schedule it
                     * with the command scheduler and remove it from the front
                     * of the event commands list.
                     */
                    if (eventCommands.size() != 0 && eventCommands.get(0).getFirst() <= trajectoryState.timeSeconds) {
                        CommandScheduler.getInstance().schedule(eventCommands.remove(0).getSecond());
                    }

                    // Update the current time.
                    currTime = Timer.getFPGATimestamp();
                }

                // Set the swerve subsystem to stop after the path has finished.
                Swerve.getInstance().setVoltage(0.0);
            },
            Swerve.getInstance()
        );
    }

    public static Command getSpeakerScoreAuto() {
        return new SequentialCommandGroup(
            ShooterCommands.getShootNoteCommand(),
            followPathAuto(AutoConstants.SPEAKER_SCORE_AUTO_PATH_FILE_NAME)
        );
    }
}
