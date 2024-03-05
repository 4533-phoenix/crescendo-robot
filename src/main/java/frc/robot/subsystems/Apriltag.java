package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApriltagConstants;

/**
 * The class for photonlib pose estimator.
 */
public final class Apriltag extends SubsystemBase {
    /**
     * The instance of the {@link Apriltag} class.
     */
    private static Apriltag apriltag = null;

    /**
     * Array of PhotonCameras
     */
    private static PhotonPoseEstimator[] estimators;

    /**
     * Gets the instance of the {@link Apriltag} class.
     * 
     * @return The instance of the {@link Apriltag} class.
     */
    public static Apriltag getInstance() {
        if (apriltag == null) {
            apriltag = new Apriltag();
        }

        return apriltag;
    }

    /**
     * The constructor for the {@link Amp} class.
     */
    private Apriltag() {
        estimators = new PhotonPoseEstimator[ApriltagConstants.PHOTON_CAMERAS.length];

        for (int i = 0; i < estimators.length; i++) {
            PhotonCamera camera = new PhotonCamera(ApriltagConstants.PHOTON_CAMERAS[i].getName());
            estimators[i] = new PhotonPoseEstimator(ApriltagConstants.FIELD_LAYOUT,
                    ApriltagConstants.PHOTON_CAMERAS[i].getStrategy(), camera,
                    ApriltagConstants.PHOTON_CAMERAS[i].getTransform());
        }
    }

    /**
     * The periodic method for the apriltag subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        SwerveDrivePoseEstimator robotEstimator = Swerve.getInstance().getPoseEstimator();

        for (PhotonPoseEstimator estimator : estimators) {
            Optional<EstimatedRobotPose> pose = estimator.update();
            
            if (pose.isPresent()) {
                robotEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
            }
        }
    }
}