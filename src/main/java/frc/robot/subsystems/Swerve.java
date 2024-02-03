package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Constants.SwerveConstants;

public final class Swerve extends SubsystemBase {
    private static Swerve swerve = null;

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(
        SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, 
        SwerveConstants.FRONT_LEFT_STEER_MOTOR_ID, 
        SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FRONT_LEFT_STEER_MOTOR_REVERSED,
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule frontRightSwerveModule = new SwerveModule(
        SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
        SwerveConstants.FRONT_RIGHT_STEER_MOTOR_ID, 
        SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FRONT_RIGHT_STEER_MOTOR_REVERSED,
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule backLeftSwerveModule = new SwerveModule(
        SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID, 
        SwerveConstants.BACK_LEFT_STEER_MOTOR_ID, 
        SwerveConstants.BACK_LEFT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BACK_LEFT_STEER_MOTOR_REVERSED,
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule backRightSwerveModule = new SwerveModule(
        SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, 
        SwerveConstants.BACK_RIGHT_STEER_MOTOR_ID, 
        SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BACK_RIGHT_STEER_MOTOR_REVERSED,
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule[] swerveModules = new SwerveModule[]{
        frontLeftSwerveModule,
        frontRightSwerveModule,
        backLeftSwerveModule,
        backRightSwerveModule
    };

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveDrivePoseEstimator poseEstimator = null;

    private final HolonomicDriveController swerveController = new HolonomicDriveController(
        new PIDController(
            SwerveConstants.X_CONTROLLER_KP, 
            SwerveConstants.X_CONTROLLER_KI, 
            SwerveConstants.X_CONTROLLER_KD
        ), 
        new PIDController(
            SwerveConstants.Y_CONTROLLER_KP, 
            SwerveConstants.Y_CONTROLLER_KI, 
            SwerveConstants.Y_CONTROLLER_KD
        ), 
        new ProfiledPIDController(
            SwerveConstants.THETA_CONTROLLER_KP, 
            SwerveConstants.THETA_CONTROLLER_KI, 
            SwerveConstants.THETA_CONTROLLER_KD, 
            new Constraints(
                SwerveConstants.MAX_ROTATIONAL_VELOCITY, 
                SwerveConstants.MAX_ROTATIONAL_ACCELERATION
            )
        )
    );

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }

        return swerve;
    }

    private Swerve() {
        swerveController.getThetaController().enableContinuousInput(0.0, 2 * Math.PI);
    }

    public HolonomicDriveController getHolonomicDriveController() {
        return swerveController;
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void registerPoseEstimator(Pose2d initialPose) {
        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.SWERVE_DRIVE_KINEMATICS, 
            Rotation2d.fromDegrees(-gyro.getYaw()), 
            new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            }, 
            initialPose
        );
    }

    public Rotation2d getRobotAngle() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void drive() {
        Translation2d translation = new Translation2d(
            RobotContainer.getController().getLeftX(), 
            RobotContainer.getController().getLeftY()
        );
        double rotation = RobotContainer.getController().getRightX();

        double xVelocity = translation.getX() * SwerveConstants.MAX_VELOCITY;
        double yVelocity = translation.getY() * SwerveConstants.MAX_VELOCITY;
        double rotationalVelocity = rotation * SwerveConstants.MAX_ROTATIONAL_VELOCITY;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, 
            yVelocity, 
            rotationalVelocity, 
            Rotation2d.fromDegrees(-gyro.getYaw())
        );

        SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setSwerveModuleStates(swerveModuleStates);
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(swerveModuleStates[i]);
        }
    }

    public void sysIDDriveTest(Measure<Voltage> voltage) {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setDriveMotorVoltage(voltage);
        }
    }

    public void sysIDSteerTest(Measure<Voltage> voltage) {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setSteerMotorVoltage(voltage);
        }
    }

    public void sysIDDriveLog(SysIdRoutineLog log) {
        for (int i = 0; i < 4; i++) {
            SwerveModule swerveModule = swerveModules[i];

            String motorName = "";

            if (i == 0) {
                motorName = "Front Left Drive Motor";
            }
            else if (i == 1) {
                motorName = "Front Right Drive Motor";
            }
            else if (i == 2) {
                motorName = "Back Left Drive Motor";
            }
            else if (i == 3) {
                motorName = "Back Right Drive Motor";
            }

            log.motor(motorName)
                .voltage(Units.Volts.of(swerveModule.getDriveMotorVoltage()))
                .linearPosition(Units.Meters.of(swerveModule.getDriveMotorLinearPosition()))
                .linearVelocity(Units.MetersPerSecond.of(swerveModule.getDriveMotorLinearVelocity()));
        }
    }

    public void sysIDSteerLog(SysIdRoutineLog log) {
        for (int i = 0; i < 4; i++) {
            SwerveModule swerveModule = swerveModules[i];

            String motorName = "";

            if (i == 0) {
                motorName = "Front Left Steer Motor";
            }
            else if (i == 1) {
                motorName = "Front Right Steer Motor";
            }
            else if (i == 2) {
                motorName = "Back Left Steer Motor";
            }
            else if (i == 3) {
                motorName = "Back Right Steer Motor";
            }

            log.motor(motorName)
                .voltage(Units.Volts.of(swerveModule.getSteerMotorVoltage()))
                .angularPosition(Units.Radians.of(swerveModule.getSteerEncoderAngle()))
                .angularVelocity(Units.RadiansPerSecond.of(swerveModule.getSteerEncoderAngularVelocity()));
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            SwerveModule swerveModule = swerveModules[i];

            if (i == 0) {
                System.out.println("Front Left Encoder Angle: " + swerveModule.getSteerEncoderAngle());
            }
            else if (i == 1) {
                System.out.println("Front Right Encoder Angle: " + swerveModule.getSteerEncoderAngle());
            }
            else if (i == 2) {
                System.out.println("Back Left Encoder Angle: " + swerveModule.getSteerEncoderAngle());
            }
            else if (i == 3) {
                System.out.println("Back Right Encoder Angle: " + swerveModule.getSteerEncoderAngle());
            }
        }

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            SwerveModule swerveModule = swerveModules[i];

            swerveModulePositions[i] = swerveModule.getSwerveModulePosition();
        }

        if (poseEstimator != null) {
            poseEstimator.update(getRobotAngle(), swerveModulePositions);
        }
    }
}
