package frc.robot.commands;

import java.util.List;
import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Constants.SwerveConstants;

public final class AutoCommands {
    public static InstantCommand tutorialAuto() {
        return new InstantCommand(
            () -> {
                PathPlannerPath path = PathPlannerPath.fromPathFile("Tutorial Path");

                List<PathPoint> pathpoints = path.getAllPathPoints();
                ArrayList<Translation2d> innerPoints = new ArrayList<Translation2d>();

                for (int i = 1; i < pathpoints.size() - 1; i++) {
                    innerPoints.add(pathpoints.get(i).position);
                }

                PathPoint startPoint = pathpoints.get(0);
                PathPoint endPoint = pathpoints.get(pathpoints.size() - 1);

                Pose2d startPose = new Pose2d(startPoint.position.getX(), startPoint.position.getY(), startPoint.position.getAngle());
                Pose2d endPose = new Pose2d(endPoint.position.getX(), endPoint.position.getY(), endPoint.position.getAngle());  

                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(SwerveConstants.MAX_VELOCITY, SwerveConstants.MAX_ACCELERATION);

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    startPose, 
                    innerPoints, 
                    endPose, 
                    trajectoryConfig
                );

                double currTime = Timer.getFPGATimestamp();

                double elapsedTime = currTime + trajectory.getTotalTimeSeconds();
                
                // Follow the trajectory.
                while (currTime <= elapsedTime) {
                    Trajectory.State trajectoryState = trajectory.sample(trajectory.getTotalTimeSeconds() - (elapsedTime - currTime));

                    double trajectoryVelocity = trajectoryState.velocityMetersPerSecond;
                    Rotation2d trajectoryRotation = trajectoryState.poseMeters.getRotation();
                    Rotation2d trajectoryHeading = new Rotation2d();

                    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        trajectoryVelocity * trajectoryRotation.getCos(),
                        trajectoryVelocity * trajectoryRotation.getSin(), 
                        0.0, 
                        Swerve.getInstance().getRobotAngle()
                    );

                    ChassisSpeeds pidValue = ChassisSpeeds.fromFieldRelativeSpeeds(
                        Swerve.getInstance().getHolonomicDriveController().calculate(
                            Swerve.getInstance().getRobotPose(), 
                            trajectoryState.poseMeters, 
                            trajectoryVelocity, 
                            trajectoryHeading
                        ),
                        Swerve.getInstance().getRobotAngle()
                    );

                    ChassisSpeeds adjustedChassisSpeeds = chassisSpeeds.plus(pidValue);

                    SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);

                    Swerve.getInstance().setSwerveModuleStates(swerveModuleStates);

                    currTime = Timer.getFPGATimestamp();
                }

                // Stop the robot.
                Swerve.getInstance().setSwerveModuleStates(SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
            },
            Swerve.getInstance()
        );
    }
}
