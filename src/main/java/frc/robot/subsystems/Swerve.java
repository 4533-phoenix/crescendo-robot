package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

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
        SwerveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED
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
        SwerveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED
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
        SwerveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED
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
        SwerveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED
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
     * The robot's gyroscope.
     */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    /**
     * The swerve drive position estimator.
     */
    private SwerveDrivePoseEstimator poseEstimator = null;

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
        swerveController.getThetaController().enableContinuousInput(0.0, (2 * Math.PI));
    }

    /**
     * Gets the swerve module from the swerve modules array corresponding
     * to the given index.
     * 
     * @param index The index of the swerve module to get from the swerve modules array.
     * @return The swerve module at the given index in the swerve modules array.
     */
    public SwerveModule getSwerveModule(int index) {
        return swerveModules[index];
    }

    /**
     * Gets the swerve drive holonomic drive controller.
     * 
     * @return The swerve drive holonomic drive controller.
     */
    public HolonomicDriveController getHolonomicDriveController() {
        return swerveController;
    }

    /**
     * Resets the robot's gyroscope yaw to zero.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Registers the swerve drive position estimator. This is done
     * because the initial position of the robot is only known
     * at the beginning of the autonomous period.
     * 
     * @param initialPose The initial position of the robot.
     */
    public void registerPoseEstimator(Pose2d initialPose) {
        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.SWERVE_DRIVE_KINEMATICS, 
            getRobotAngle(), 
            new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            }, 
            initialPose
        );
    }

    /**
     * Returns the robot's accumulated yaw angle.
     * 
     * @return The robot's accumulated yaw angle.
     */
    public Rotation2d getRobotAngle() {
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
        /*
         * Get the velocities as the x, y, and rotation velocity factors
         * multiplied by their respective velocities.
         */
        double xVelocity = x * SwerveConstants.MAX_VELOCITY;
        double yVelocity = y * SwerveConstants.MAX_VELOCITY;
        double rotationalVelocity = rotation * SwerveConstants.MAX_ROTATIONAL_VELOCITY;

        /*
         * Get field relative chassis speeds from the velocities and
         * current robot angle.
         */
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, 
            yVelocity, 
            rotationalVelocity, 
            getRobotAngle()
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

    /**
     * The periodic method for the swerve drive subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        // for (int i = 0; i < swerveModules.length; i++) {
        //     SwerveModule swerveModule = swerveModules[i];

        //     if (i == 0) {
        //         System.out.println("Front Left Angle: " + swerveModule.getSteerEncoderAngle());
        //     }
        //     else if (i == 1) {
        //         System.out.println("Front Right Angle: " + swerveModule.getSteerEncoderAngle());
        //     }
        //     else if (i == 2) {
        //         System.out.println("Back Left Angle: " + swerveModule.getSteerEncoderAngle());
        //     }
        //     else if (i == 3) {
        //         System.out.println("Back Right Angle: " + swerveModule.getSteerEncoderAngle());
        //     }
        // }

        // Create an array to store the current swerve module positions.
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

        /*
         * Loop over the swerve modules and set the corresponding current
         * swerve module positions.
         */
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];

            swerveModulePositions[i] = swerveModule.getSwerveModulePosition();
        }

        /*
         * If the swerve drive position estimator has been registered,
         * then update it with the current robot angle and the current
         * swerve module positions.
         */
        if (poseEstimator != null) {
            poseEstimator.update(getRobotAngle(), swerveModulePositions);
        }
    }
}
