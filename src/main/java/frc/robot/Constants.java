package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class SwerveModuleConstants {
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0); // m
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // m

        public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0;

        public static final double DRIVE_MOTOR_REVOLUTIONS_TO_METERS = 
            WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO; // m / rot

        public static final double DRIVE_MOTOR_RPM_TO_METERS_PER_SECOND = 
            WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 60.0); // m/s / rpm

        public static final double DRIVE_MOTOR_VELOCITY_DEADBAND = 0.2; // m/s

        public static final double STEER_MOTOR_MAX_VELOCITY = 30.0 * Math.PI; // rad/s
        public static final double STEER_MOTOR_MAX_ACCELERATION = 60.0 * Math.PI; // rad/s^2
    }

    public static final class SwerveConstants {
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5); // m

        public static final double WHEEL_BASE = Units.inchesToMeters(20.5); // m

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = 
            new SwerveDriveKinematics(
                new Translation2d[]{
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)});
        
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 8;

        public static final boolean FRONT_LEFT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_MOTOR_REVERSED = true;

        public static final boolean FRONT_LEFT_STEER_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_STEER_MOTOR_REVERSED = false;
        public static final boolean BACK_LEFT_STEER_MOTOR_REVERSED = false;
        public static final boolean BACK_RIGHT_STEER_MOTOR_REVERSED = false;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 9;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 10;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 11;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 12;
        
        public static final double FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 0.12885; // rad
        public static final double FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 6.16660; // rad
        public static final double BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 2.13684; // rad
        public static final double BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 0.90045; // rad

        public static final boolean FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;

        public static final double FRONT_LEFT_DRIVE_MOTOR_KS = 0.30756; // V
        public static final double FRONT_RIGHT_DRIVE_MOTOR_KS = 0.24138; // V
        public static final double BACK_LEFT_DRIVE_MOTOR_KS = 0.31912; // V
        public static final double BACK_RIGHT_DRIVE_MOTOR_KS = 0.20699; // V

        public static final double FRONT_LEFT_DRIVE_MOTOR_KV = 2.3401; // V / m/s
        public static final double FRONT_RIGHT_DRIVE_MOTOR_KV = 2.5475; // V / m/s
        public static final double BACK_LEFT_DRIVE_MOTOR_KV = 2.5673; // V / m/s
        public static final double BACK_RIGHT_DRIVE_MOTOR_KV = 2.536; // V / m/s

        public static final double FRONT_LEFT_DRIVE_MOTOR_KP = 0.0; // V / m/s
        public static final double FRONT_RIGHT_DRIVE_MOTOR_KP = 0.0; // V / m/s
        public static final double BACK_LEFT_DRIVE_MOTOR_KP = 0.0; // V / m/s
        public static final double BACK_RIGHT_DRIVE_MOTOR_KP = 0.0; // V / m/s

        public static final double FRONT_LEFT_STEER_MOTOR_KP = 3.5; // V / rad
        public static final double FRONT_RIGHT_STEER_MOTOR_KP = 4.0; // V / rad
        public static final double BACK_LEFT_STEER_MOTOR_KP = 4.5; // V / rad
        public static final double BACK_RIGHT_STEER_MOTOR_KP = 4.5; // V / rad

        public static final double FRONT_LEFT_DRIVE_MOTOR_KI = 1.0; // V / ∫ v(t) dt
        public static final double FRONT_RIGHT_DRIVE_MOTOR_KI = 0.08; // V / ∫ v(t) dt
        public static final double BACK_LEFT_DRIVE_MOTOR_KI = 0.075; // V / ∫ v(t) dt
        public static final double BACK_RIGHT_DRIVE_MOTOR_KI = 0.075; // V / ∫ v(t) dt

        public static final double FRONT_LEFT_STEER_MOTOR_KI = 0.01; // V / ∫ θ(t) dt
        public static final double FRONT_RIGHT_STEER_MOTOR_KI = 0.01; // V / ∫ θ(t) dt
        public static final double BACK_LEFT_STEER_MOTOR_KI = 0.01; // V / ∫ θ(t) dt
        public static final double BACK_RIGHT_STEER_MOTOR_KI = 0.01; // V / ∫ θ(t) dt

        public static final double FRONT_LEFT_DRIVE_MOTOR_KD = 0.0; // V / a(t)
        public static final double FRONT_RIGHT_DRIVE_MOTOR_KD = 0.0; // V / a(t)
        public static final double BACK_LEFT_DRIVE_MOTOR_KD = 0.0; // V / a(t)
        public static final double BACK_RIGHT_DRIVE_MOTOR_KD = 0.0; // V / a(t)

        public static final double FRONT_LEFT_STEER_MOTOR_KD = 0.075; // V / ω(t)
        public static final double FRONT_RIGHT_STEER_MOTOR_KD = 0.075; // V / ω(t)
        public static final double BACK_LEFT_STEER_MOTOR_KD = 0.075; // V / ω(t)
        public static final double BACK_RIGHT_STEER_MOTOR_KD = 0.075; // V / ω(t)

        public static final double X_CONTROLLER_KP = 0.08; // m/s / m
        public static final double X_CONTROLLER_KI = 0.0; // m/s / ∫ s(t) dt
        public static final double X_CONTROLLER_KD = 0.0; // m/s / v(t)

        public static final double Y_CONTROLLER_KP = 0.08; // m/s / m
        public static final double Y_CONTROLLER_KI = 0.0; // m/s / ∫ s(t) dt
        public static final double Y_CONTROLLER_KD = 0.0; // m/s / v(t)

        public static final double THETA_CONTROLLER_KP = 2.2; // rad/s / rad
        public static final double THETA_CONTROLLER_KI = 0.0; // rad/s / ∫ θ(t) dt
        public static final double THETA_CONTROLLER_KD = 0.0; // rad/s / ω(t)

        public static final double MAX_VELOCITY = 4.4; // m/s
        public static final double MAX_ACCELERATION = 3.0; // m/s^2

        public static final double SLOW_VELOCITY = 1.0; // m/s

        public static final double MAX_ROTATIONAL_VELOCITY = Math.PI; // rad/s
        public static final double MAX_ROTATIONAL_ACCELERATION = Math.PI; // rad/s^2

        public static final double TRACK_NOTE_OFFSET_DEADBAND = 0.08;
        public static final double TRACK_NOTE_ROTATIONAL_VELOCITY = Math.PI / 4.0; // rad/s
        public static final double TRACK_NOTE_SLOW_ROTATIONAL_VELOCITY = Math.PI / 6.0; // rad/s
        
        public static final double ACQUIRE_NOTE_LINEAR_VELOCITY = 1.0; // m/s
        public static final double ACQUIRE_NOTE_TIMEOUT = 5.0; // s

        public static final double NOTE_DIMENSIONS_DEADBAND = 0.0; // px^2
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 13;

        public static final double INTAKE_MOTOR_VOLTAGE = 12.0; // V
    }

    public static final class ShooterConstants {
        public static final int LIFT_MOTOR_ID = 14;
        public static final int LEFT_SHOOTER_MOTOR_ID = 19;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 15;

        public static final int SHOOTER_LIMIT_SWITCH_ID = 0;

        public static final double LIFT_MOTOR_VOLTAGE = 9.0; // V
        public static final double SHOOTER_MOTOR_VOLTAGE = 12.0; // V
    }

    public static final class ClimbConstants {
        public static final int LEFT_CLIMB_MOTOR_ID = 16;
        public static final int RIGHT_CLIMB_MOTOR_ID = 17;

        public static final int LEFT_CLIMB_LIMIT_SWITCH_ID = 2;
        public static final int RIGHT_CLIMB_LIMIT_SWITCH_ID = 1;

        public static final double CLIMB_MOTOR_VOLTAGE = 11.0; // V

        public static final double CLIMB_CURRENT_DEADBAND = 0.0; // amps
    }

    public static final class AmpConstants {
        public static final int AMP_MOTOR_ID = 18;

        public static final int AMP_LIMIT_SWITCH_ID = 3;

        public static final double AMP_MOTOR_VOLTAGE = 4.0; // V

        public static enum AMP_POSITION {
            RECEIVE_POSITION,
            DROP_POSITION
        }

        public static enum AMP_MOVEMENT_DIRECTION {
            TOWARDS_RECEIVE_POSITION,
            TOWARDS_DROP_POSITION
        }

        public static final double AMP_POSITION_DELAY = 0.1; // s
    }

    public static final class ApriltagConstants {
        public static final class ApriltagCameraConfig {
            private String name;
            private Transform3d transform;
            private PoseStrategy strategy;

            public ApriltagCameraConfig(
                    String name, 
                    Transform3d transform, 
                    PoseStrategy strategy) {
                this.name = name;
                this.transform = transform;
                this.strategy = strategy;
            }

            public String getName() {
                return name;
            }

            public Transform3d getTransform() {
                return transform;
            }

            public PoseStrategy getStrategy() {
                return strategy;
            }
        }

        public static final ApriltagCameraConfig[] PHOTON_CAMERAS = {
            new ApriltagCameraConfig(
                "Front_Camera", 
                new Transform3d(
                    new Translation3d(
                        0.03, 
                        0.57785, 
                        0.5), 
                    new Rotation3d(
                        0, 
                        0, 
                        0)), 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
            new ApriltagCameraConfig(
                "Back_Camera", 
                new Transform3d(
                    new Translation3d(
                        -0.3175, 
                        0.0, 
                        0.60325), 
                    new Rotation3d(
                        0.0, 
                        -Math.PI / 6.0, 
                        Math.PI)), 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
        };

        public static final AprilTagFieldLayout FIELD_LAYOUT = 
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static final class LimelightConstants {
        public static final String LIMELIGHT_NAME = "limelight-front";
    }

    public static final class AutoConstants {
        public static final double FIELD_LENGTH = 16.54; // m

        public static final double MAX_VELOCITY = 4.0; // m/s
        public static final double MAX_ACCELERATION = 3.0; // m/s^2
        public static final double MAX_ROTATIONAL_VELOCITY = Math.PI; // rad/s
        public static final double MAX_ROTATIONAL_ACCELERATION = 2.0 * Math.PI; // rad/s^2

        // Do nothing autonomous constants.
        public static final String DO_NOTHING_AUTO_KEY = "Do Nothing Auto";

        // Source exit autonomous constants.
        public static final String SOURCE_EXIT_AUTO_KEY = "Source Exit Auto";
        public static final String SOURCE_EXIT_AUTO_PATH_FILE_NAME = "Source Exit Auto";

        // Only shoot autonomous constants.
        public static final String ONLY_SHOOT_AUTO_KEY = "Only Shoot Auto";

        // Amp exit autonomous constants.
        public static final String AMP_SCORE_AUTO_KEY = "Amp Score Auto";
        public static final String AMP_SCORE_AUTO_PATH_FILE_NAME = "Amp Score Auto";

        // Speaker score autonomous constants.
        public static final String SPEAKER_SCORE_AUTO_KEY = "Speaker Score Auto";
        public static final String SPEAKER_SCORE_AUTO_PATH_FILE_NAME = "Speaker Score Auto";

        // Speaker score to center autonomous constants.
        public static final String SPEAKER_SCORE_TO_CENTER_AUTO_KEY = 
            "Speaker Score To Center Auto";
        
        public static final String SPEAKER_SCORE_TO_CENTER_AUTO_PATH_FILE_NAME =
            "Speaker Score To Center Auto";

        // Double speaker score autonomous constants.
        public static final String DOUBLE_SPEAKER_SCORE_AUTO_KEY = "Double Speaker Score Auto";

        // Triple speaker score autonomous constants.
        public static final String TRIPLE_SPEAKER_SCORE_AUTO_KEY = "Triple Speaker Score Auto";
        public static final String TOP_NOTE_PATH = "Top Note Path";

        // Pathplanner commands.
        public static final String INTAKE_NOTE_COMMAND = "Intake Note Command";
        
        public static final Pose2d SUBWOOFER_CENTER_POSE =
            new Pose2d(1.32, 5.52, new Rotation2d());
        
        public static final Pose2d SUBWOOFER_SOURCE_SIDE_POSE =
            new Pose2d(0.81, 4.47, new Rotation2d((5.0 * Math.PI) / 3.0));
        
        public static final Pose2d SUBWOOFER_AMP_SIDE_POSE =
            new Pose2d(0.81, 6.62, new Rotation2d(Math.PI / 3.0));
        
        public static final Pose2d SOURCE_POSE =
            new Pose2d(14.85, 0.60, new Rotation2d((2.0 * Math.PI) / 3.0));
        
        public static final Pose2d AMP_POSE =
            new Pose2d(1.83, 7.68, new Rotation2d((3.0 * Math.PI) / 2.0));
        
        public static final Pose2d[] SHOOTER_POINTS_OF_INTEREST =
            new Pose2d[]{
                SUBWOOFER_CENTER_POSE,
                SUBWOOFER_SOURCE_SIDE_POSE,
                SUBWOOFER_AMP_SIDE_POSE};
        
        public static final Pose2d[] DEFAULT_POINTS_OF_INTEREST =
            new Pose2d[]{
                SOURCE_POSE,
                AMP_POSE};

        public static final double POINT_OF_INTEREST_DISTANCE_DEADBAND = 4.0; // m
    }

    public static final class ControllerConstants {
        // Drive controller IDs.
        public static final int DRIVER_CONTROLLER_ID = 0;
        public static final int MANIPULATOR_CONTROLLER_ID = 1;

        // Drive controller button IDs.
        public static final int BUTTON_A = 1;
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int BUTTON_LB = 5;
        public static final int BUTTON_RB = 6;
        public static final int BUTTON_BACK = 7;
        public static final int BUTTON_START = 8;
        public static final int LEFT_STICK_PRESS_DOWN = 9;
        public static final int RIGHT_STICK_PRESS_DOWN = 10;

        // Drive controller axis IDs.
        public static final int LEFT_STICK_AXIS = 1;
        public static final int RIGHT_STICK_AXIS = 5;
        public static final int LEFT_TRIGGER_AXIS = 2;
        public static final int RIGHT_TRIGGER_AXIS = 3;

        // Defines the deadzone for the controller analog inputs.
        public static final double ANALOG_INPUT_DEADBAND = 0.1;
    }

    public static final class JoystickConstants {
        public static final int DRIVER_JOYSTICK_ID = 2;
        public static final int MANIPULATOR_JOYSTICK_ID = 3;
        
        public static final int BUTTON_TRIGGER = 1;
        public static final int BUTTON_THUMB = 2;
        public static final int BUTTON_3 = 3;
        public static final int BUTTON_4 = 4;
        public static final int BUTTON_5 = 5;
        public static final int BUTTON_6 = 6;
        public static final int BUTTON_7 = 7;
        public static final int BUTTON_8 = 8;
        public static final int BUTTON_9 = 9;
        public static final int BUTTON_10 = 10;
        public static final int BUTTON_11 = 11;
        public static final int BUTTON_12 = 12;
    }
}
