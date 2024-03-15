package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteDetectorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.helpers.NoteDetector;

/**
 * The class for the swerve drive subsystem.
 */
public final class Swerve extends SubsystemBase {
    /**
     * The instance of the {@link Swerve} class.
     */
    private static Swerve swerve = null;

    /**
     * The front left swerve module.
     */
    private final SwerveModule frontLeftSwerveModule = new SwerveModule(
        SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, 
        SwerveConstants.FRONT_LEFT_STEER_MOTOR_ID, 
        SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FRONT_LEFT_STEER_MOTOR_REVERSED,
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED,
        new SimpleMotorFeedforward(
            SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_KS, 
            SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_KV),
        new PIDController(
            SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_KP, 
            SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_KI, 
            SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_KD),
        new ProfiledPIDController(
            SwerveConstants.FRONT_LEFT_STEER_MOTOR_KP, 
            SwerveConstants.FRONT_LEFT_STEER_MOTOR_KI, 
            SwerveConstants.FRONT_LEFT_STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION))
    );

    /**
     * The front right swerve module.
     */
    private final SwerveModule frontRightSwerveModule = new SwerveModule(
        SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
        SwerveConstants.FRONT_RIGHT_STEER_MOTOR_ID, 
        SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FRONT_RIGHT_STEER_MOTOR_REVERSED,
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED,
        new SimpleMotorFeedforward(
            SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_KS, 
            SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_KV),
        new PIDController(
            SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_KP, 
            SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_KI, 
            SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_KD),
        new ProfiledPIDController(
            SwerveConstants.FRONT_RIGHT_STEER_MOTOR_KP, 
            SwerveConstants.FRONT_RIGHT_STEER_MOTOR_KI, 
            SwerveConstants.FRONT_RIGHT_STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION))
    );

    /**
     * The back left swerve module.
     */
    private final SwerveModule backLeftSwerveModule = new SwerveModule(
        SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID, 
        SwerveConstants.BACK_LEFT_STEER_MOTOR_ID, 
        SwerveConstants.BACK_LEFT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BACK_LEFT_STEER_MOTOR_REVERSED,
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED,
        new SimpleMotorFeedforward(
            SwerveConstants.BACK_LEFT_DRIVE_MOTOR_KS, 
            SwerveConstants.BACK_LEFT_DRIVE_MOTOR_KV),
        new PIDController(
            SwerveConstants.BACK_LEFT_DRIVE_MOTOR_KP, 
            SwerveConstants.BACK_LEFT_DRIVE_MOTOR_KI, 
            SwerveConstants.BACK_LEFT_DRIVE_MOTOR_KD),
        new ProfiledPIDController(
            SwerveConstants.BACK_LEFT_STEER_MOTOR_KP, 
            SwerveConstants.BACK_LEFT_STEER_MOTOR_KI, 
            SwerveConstants.BACK_LEFT_STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION))
    );

    /**
     * The back right swerve module.
     */
    private final SwerveModule backRightSwerveModule = new SwerveModule(
        SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, 
        SwerveConstants.BACK_RIGHT_STEER_MOTOR_ID, 
        SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BACK_RIGHT_STEER_MOTOR_REVERSED,
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID, 
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET, 
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED,
        new SimpleMotorFeedforward(
            SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_KS, 
            SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_KV),
        new PIDController(
            SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_KP, 
            SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_KI, 
            SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_KD),
        new ProfiledPIDController(
            SwerveConstants.BACK_RIGHT_STEER_MOTOR_KP, 
            SwerveConstants.BACK_RIGHT_STEER_MOTOR_KI, 
            SwerveConstants.BACK_RIGHT_STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION))
    );

    /**
     * The array that contains the swerve modules.
     * <br></br>
     * The order is front left, front right, back left, back right.
     */
    private final SwerveModule[] swerveModules = new SwerveModule[]{
        frontLeftSwerveModule,
        frontRightSwerveModule,
        backLeftSwerveModule,
        backRightSwerveModule
    };
    
    /**
     * Whether or not the swerve drive subsystem
     * is in slow mode.
     */
    private boolean isSlow = false;

    /**
     * Whether or not the swerve drive subsystem
     * is in robot relative mode.
     */
    private boolean isRobotRelative = false;

    /**
     * The robot's gyroscope.
     */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    /**
     * The swerve drive position estimator.
     */
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.SWERVE_DRIVE_KINEMATICS, 
        getGyroAngle(), 
        getSwerveModulePositions(),
        new Pose2d()
    );

    /**
     * The swerve drive holonomic drive controller.
     */
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
    
    /**
     * The note detector for the swerve drive subsystem.
     */
    private final NoteDetector noteDetector = new NoteDetector(NoteDetectorConstants.NOTE_DETECTOR_NAME);

    /**
     * Gets the instance of the {@link Swerve} class.
     * 
     * @return The instance of the {@link Swerve} class.
     */
    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }

        return swerve;
    }

    /**
     * The constructor for the {@link Swerve} class.
     */
    private Swerve() {
        // Set continous input for the theta controller.
        swerveController.getThetaController().enableContinuousInput(0.0, (2.0 * Math.PI));
    }

    /**
     * Gets the swerve modules of the swerve drive subsystem.
     * 
     * @return The swerve modules of the swerve drive subsystem.
     */
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    /**
     * Gets the swerve module from the swerve modules array corresponding
     * to the given index.
     * 
     * @param index The index of the swerve module to get from the swerve modules array.
     * 
     * @return The swerve module at the given index in the swerve modules array.
     */
    public SwerveModule getSwerveModule(int index) {
        return swerveModules[index];
    }

    /**
     * Gets the current chassis speeds of the swerve drive subsystem.
     * 
     * @return The current chassis speeds of the swerve drive subsystem.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * Gets the current swerve module states of the
     * swerve drive subsystem swerve modules.
     * 
     * @return The current swerve module states of the
     * swerve drive subsystem swerve modules.
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModuleStates[i] = swerveModules[i].getSwerveModuleState();
        }

        return swerveModuleStates;
    }

    /**
     * Gets the current swerve module positions of the 
     * swerve drive subsystem swerve modules.
     * 
     * @return The current swerve module positions of the 
     * swerve drive subsystem swerve modules.
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
        }

        return swerveModulePositions;
    }

    /**
     * Gets the swerve drive holonomic drive controller.
     * 
     * @return The swerve drive holonomic drive controller.
     */
    public HolonomicDriveController getHolonomicDriveController() {
        return swerveController;
    }

    public NoteDetector getNoteDetector() {
        return noteDetector;
    }

    /**
     * Resets the robot's gyroscope yaw to zero.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Resets the swerve drive position estimator. This is done
     * because the initial position of the robot is only known
     * during the autonomous period.
     * 
     * @param initialPose The initial position of the robot.
     */
    public void resetPoseEstimator(Pose2d initialPose) {
        poseEstimator.resetPosition(
            getGyroAngle(), 
            getSwerveModulePositions(), 
            initialPose
        );
    }

    /**
     * Returns the robot's accumulated yaw angle.
     * 
     * @return The robot's accumulated yaw angle.
     */
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**
     * Returns the robot position reported by the swerve drive
     * position estimator.
     * 
     * @return The robot position reported by the swerve drive
     * position estimator.
     */
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the Pose Estimator
     * 
     * @return the robot pose estimator
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void setSlowMode(boolean isSlow) {
        this.isSlow = isSlow;
    }

    public void toggleRobotRelativeMode() {
        isRobotRelative = !isRobotRelative;
    }

    /**
     * This method is used mainly for driving the swerve drive
     * subsystem during the teleoperated period of the match.
     * <br></br>
     * It takes in x, y, and rotation velocity factors and sets
     * the swerve drive subsystem to drive at the resulting
     * velocities.
     * 
     * @param x The x velocity factor.
     * @param y The y velocity factor.
     * @param rotation The rotational velocity factor.
     */
    public void drive(double x, double y, double rotation) {
        // Get the current alliance from driver station.
        Optional<Alliance> driverStationAlliance = DriverStation.getAlliance();

        /*
         * If the current alliance from driver station is
         * not present, then set the alliance to blue
         * alliance, and if it is, set the alliance
         * to the current alliance from driver station.
         */
        Alliance alliance = !driverStationAlliance.isPresent() 
            ? Alliance.Blue
            : driverStationAlliance.get(); 

        /*
         * If the swerve drive subsystem is in slow mode,
         * then set the velocity to the slow mode velocity,
         * and if it is not, then set it to the max velocity.
         */
        double velocity = isSlow ? SwerveConstants.SLOW_VELOCITY : SwerveConstants.MAX_VELOCITY;

        /*
         * If the alliance is the red alliance, then reverse
         * the velocity, and if it is not, keep it the same.
         */
        velocity *= (alliance == Alliance.Red && !isRobotRelative) ? -1.0 : 1.0;

        /*
         * Get the velocities as the x, y, and rotation velocity factors
         * multiplied by their respective velocities.
         */
        double xVelocity = x * velocity;
        double yVelocity = y * velocity;
        double rotationalVelocity = rotation * SwerveConstants.MAX_ROTATIONAL_VELOCITY;

        /*
         * If the swerve drive subsystem is in robot relative mode,
         * then get the robot relative chassis speeds from the velocities
         * and current robot angle, and if it is not, then get the field 
         * relative chassis speeds from the velocities and 
         * current robot angle.
         */
        ChassisSpeeds chassisSpeeds = isRobotRelative
            ? new ChassisSpeeds(
                xVelocity, 
                yVelocity, 
                rotationalVelocity
            )
            : ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, 
                yVelocity, 
                rotationalVelocity, 
                getRobotPose().getRotation()
            );

        // Convert the chassis speeds to swerve module states.
        SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Set the swerve drive subsystem to drive at the swerve module states.
        setSwerveModuleStates(swerveModuleStates);
    }

    /**
     * Sets the swerve drive subsystem to drive at the given
     * swerve module states.
     * 
     * @param swerveModuleStates The swerve module states for the
     * swerve drive subsystem to drive at.
     */
    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        /*
         * Loop over the swerve modules and set the corresponding
         * swerve module states.
         */
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(swerveModuleStates[i]);
        }
    }

    /**
     * The drive test method for the SysID drive routine.
     * Sets the drive motors of each swerve module to the 
     * given voltage. 
     * 
     * @param voltage The voltage to set each swerve module
     * drive motor to.
     */
    public void sysIDDriveTest(Measure<Voltage> voltage) {
        /*
         * Loop over the swerve modules and set their
         * drive motors to the given voltage.
         */
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setDriveMotorVoltage(voltage);
        }
    }

    /**
     * The steer test method for the SysID steer routine.
     * Sets the steer motors of each swerve module to the
     * given voltage.
     * 
     * @param voltage The voltage to set each swerve module
     * steer motor to.
     */
    public void sysIDSteerTest(Measure<Voltage> voltage) {
        /*
         * Loop over the swerve modules and set their
         * steer motors to the given voltage.
         */
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setSteerMotorVoltage(voltage);
        }
    }

    /**
     * The drive log method for the SysID drive routine.
     * Logs the voltage, linear position, and linear velocity
     * of each swerve module drive motor to the given SysID log.
     * 
     * @param log The SysID log that records the voltage,
     * linear position, and linear velocity of each swerve
     * module drive motor.
     */
    public void sysIDDriveLog(SysIdRoutineLog log) {
        /*
         * Loop over the swerve modules and log their
         * drive motor voltage, linear position,
         * and linear velocity.
         */
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];

            String motorName = "";

            if (i == 0) {
                motorName = "Front Left Drive Motor";
            } else if (i == 1) {
                motorName = "Front Right Drive Motor";
            } else if (i == 2) {
                motorName = "Back Left Drive Motor";
            } else if (i == 3) {
                motorName = "Back Right Drive Motor";
            }

            log.motor(motorName)
                .voltage(Units.Volts.of(swerveModule.getDriveMotorVoltage()))
                .linearPosition(Units.Meters.of(swerveModule.getDriveMotorLinearPosition()))
                .linearVelocity(Units.MetersPerSecond.of(swerveModule.getDriveMotorLinearVelocity()));
        }
    }

    /**
     * The steer log method for the SysID steer routine.
     * Logs the voltage, angular position, and angular velocity
     * of each swerve module steer motor to the given SysID log.
     * 
     * @param log The SysID log that records the voltage,
     * angular position, and angular velocity of each swerve
     * module steer motor.
     */
    public void sysIDSteerLog(SysIdRoutineLog log) {
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];

            String motorName = "";

            if (i == 0) {
                motorName = "Front Left Steer Motor";
            } else if (i == 1) {
                motorName = "Front Right Steer Motor";
            } else if (i == 2) {
                motorName = "Back Left Steer Motor";
            } else if (i == 3) {
                motorName = "Back Right Steer Motor";
            }

            log.motor(motorName)
                .voltage(Units.Volts.of(swerveModule.getSteerMotorVoltage()))
                .angularPosition(Units.Radians.of(swerveModule.getSteerEncoderAngle()))
                .angularVelocity(Units.RadiansPerSecond.of(swerveModule.getSteerEncoderAngularVelocity()));
        }
    }

    /**
     * The periodic method for the swerve drive subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        /*
         * Update the swerve drive position estimator 
         * with the current robot angle and the current
         * swerve module positions.
         */
        poseEstimator.update(getGyroAngle(), getSwerveModulePositions());
    }
}
