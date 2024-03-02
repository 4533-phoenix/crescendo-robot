package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.AutoConstants.ALLIANCE;
import frc.robot.subsystems.Swerve;

/**
 * The class for the auto commands.
 */
public final class AutoCommands {
    /**
     * Gets an auto command that follows the path corresponding to the given
     * path file name given the initial chassis speeds and the initial rotation.
     * 
     * @param pathFile The path file name.
     * @param initialChassisSpeeds The initial chassis speeds.
     * @param initialRotation The initial rotation.
     * 
     * @return An auto command that follows the path corresponding to the given
     * path file name given the initial chassis speeds and the initial rotation.
     */
    public static Command followPathAuto(String pathFile, ChassisSpeeds initialChassisSpeeds, Rotation2d initialRotation, ALLIANCE alliance) {
        /*
         * Check if the path file exists. If it does not,
         * then print the stack trace of the error and
         * return an empty instant command.
         */
        try {
            PathPlannerPath.fromPathFile(pathFile);
        } catch (Exception e) {
            e.printStackTrace();

            return new InstantCommand();
        }

        return new InstantCommand(
            () -> {
                // Get the path from the path file.
                PathPlannerPath path = alliance == ALLIANCE.RED_ALLIANCE 
                    ? PathPlannerPath.fromPathFile(pathFile).flipPath()
                    : PathPlannerPath.fromPathFile(pathFile);

                // Create the trajectory from the path.
                PathPlannerTrajectory trajectory = path.getTrajectory(initialChassisSpeeds, initialRotation);
                
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

                    // Update the current time.
                    currTime = Timer.getFPGATimestamp();
                }

                // Set the swerve subsystem to stop after the path has finished.
                Swerve.getInstance().setSwerveModuleStates(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
            },
            Swerve.getInstance()
        );
    }
}
