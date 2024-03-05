package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * The class for a swerve module.
 */
public final class SwerveModule {
    /**
     * The swerve module drive motor.
     */
    private CANSparkMax driveMotor;

    /**
     * The swerve module steer motor.
     */
    private CANSparkMax steerMotor;

    /**
     * The swerve module drive encoder.
     */
    private RelativeEncoder driveEncoder;

    /**
     * The swerve module steer encoder.
     */
    private CANcoder steerEncoder;

    /**
     * The swerve module steer encoder offset.
     */
    private double steerEncoderOffset;

    /**
     * Whether or not the swerve module
     * steer encoder is reversed.
     */
    private boolean steerEncoderReversed;

    /**
     * The swerve module drive feedforward.
     */
    private SimpleMotorFeedforward driveFeedforward;

    /**
     * The swerve module drive PID controller.
     */
    private PIDController drivePIDController;

    /**
     * The swerve module steer PID controller.
     */
    private ProfiledPIDController steerPIDController;

    /**
     * The constructor for the {@link SwerveModule} class.
     * Constructs a swerve module given swerve module drive 
     * and steer motor IDs, whether or not the swerve module 
     * drive and steer motors are reversed, the swerve module 
     * steer encoder ID, the swerve module steer encoder
     * offset, whether or not the swerve module steer 
     * encoder is reversed, the swerve module drive
     * feedfoward, the swerve module drive PID controller,
     * and the swerve module steer PID controller.
     * 
     * @param driveMotorID The swerve module drive motor ID.
     * @param steerMotorID The swerve module steer motor ID.
     * @param driveMotorReversed Whether or not the swerve module
     * drive motor is reversed.
     * @param steerMotorReversed Whether or not the swerve module
     * steer motor is reversed.
     * @param steerEncoderID The swerve module steer encoder ID.
     * @param steerEncoderOffset The swerve module steer encoder
     * offset.
     * @param steerEncoderReversed Whether or not the swerve module
     * steer encoder is reversed.
     * @param driveFeedforward The swerve module drive feedforward.
     * @param drivePIDController The swerve module drive PID
     * controller.
     * @param steerPIDController The swerve module steer PID
     * controller.
     */
    public SwerveModule(
            int driveMotorID, 
            int steerMotorID, 
            boolean driveMotorReversed, 
            boolean steerMotorReversed, 
            int steerEncoderID, 
            double steerEncoderOffset, 
            boolean steerEncoderReversed,
            SimpleMotorFeedforward driveFeedforward,
            PIDController drivePIDController,
            ProfiledPIDController steerPIDController) {
        // Create the swerve module drive and steer motors.
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        
        // Get the swerve module drive encoder.
        driveEncoder = driveMotor.getEncoder();

        /*
         * Set the inversion status of the swerve module
         * drive and steer motors.
         */
        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveMotor.setOpenLoopRampRate(SwerveModuleConstants.RAMP_RATE);

        /*
         * Set the swerve module drive encoder position
         * conversion factor to convert from rotations
         * to meters.
         */
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_REVOLUTIONS_TO_METERS);

        /*
         * Set the swerve module drive encoder velocity
         * conversion factor to convert from RPM to
         * meters per second.
         */
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_RPM_TO_METERS_PER_SECOND);
        
        // Create the swerve module steer encoder.
        steerEncoder = new CANcoder(steerEncoderID, "rio");

        /*
         * Set the swerve module steer encoder to
         * have a range of [0, 1) for rotations
         * and for the positive direction to
         * be counterclockwise.
         */
        steerEncoder.getConfigurator().apply(
            new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            )
        );

        // Set the swerve module steer encoder offset.
        this.steerEncoderOffset = steerEncoderOffset;

        /*
         * Set whether or not the swerve module steer encoder
         * is reversed.
         */
        this.steerEncoderReversed = steerEncoderReversed;

        // Set the swerve module drive feedforward.
        this.driveFeedforward = driveFeedforward;
        
        // Set the swerve module drive PID controller.
        this.drivePIDController = drivePIDController;

        // Set the swerve module steer PID controller.
        this.steerPIDController = steerPIDController;

        /*
         * Set continuous input for the swerve module 
         * steer PID controller. 
         */
        steerPIDController.enableContinuousInput(0.0, (2.0 * Math.PI));
    }

    /**
     * Gets the current voltage applied to the swerve module
     * drive motor.
     * 
     * @return The current voltage applied to the swerve module
     * drive motor.
     */
    public double getDriveMotorVoltage() {
        return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    }

    /**
     * Gets the current voltage applied to the swerve module
     * steer motor.
     * 
     * @return The current voltage applied to the swerve module
     * steer motor.
     */
    public double getSteerMotorVoltage() {
        return steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
    }

    /**
     * Gets the current linear position of the swerve module
     * drive encoder.
     * 
     * @return The current linear position of the swerve module
     * drive encoder.
     */
    public double getDriveMotorLinearPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the current linear velocity of the swerve module
     * drive encoder.
     * 
     * @return The current linear velocity of the swerve module
     * drive encoder.
     */
    public double getDriveMotorLinearVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the current angle of the swerve module
     * steer encoder.
     * 
     * @return The current angle of the swerve module
     * steer encoder.
     */
    public double getSteerEncoderAngle() {
        // Get the angle reading from the swerve module steer encoder in rotations.
        double angle = steerEncoder.getAbsolutePosition().getValueAsDouble();

        // If the swerve module steer encoder is reversed, then flip the angle.
        if (steerEncoderReversed) {
            angle = 1.0 - angle;
        }
        
        // Convert the angle into radians.
        angle *= (2.0 * Math.PI);

        // Offset the angle by the swerve module steer encoder offset.
        angle -= steerEncoderOffset;

        /*
         * If the angle is less than 0, then add 2π.
         * 
         * This is caused by the swerve module steer encoder 
         * being on a set interval of rotations of [0, 1), which
         * with an offset causes the interval to start
         * later and end earlier, causing the need
         * for adding 2π when negative angles are produced.
         */
        if (angle < 0.0) {
            angle += (2.0 * Math.PI);
        }

        return angle;
    }

    /**
     * Gets the current angular velocity of the swerve module
     * steer encoder.
     * 
     * @return The current angular velocity of the swerve module
     * steer encoder.
     */
    public double getSteerEncoderAngularVelocity() {
        // Get the angular velocity reading from the swerve module steer encoder.
        double angularVelocity = steerEncoder.getVelocity().getValueAsDouble();

        /*
         * If the swerve module steer encoder is reversed, then reverse 
         * the angular velocity.
         */
        angularVelocity *= steerEncoderReversed ? -1.0 : 1.0;

        // Convert the angular velocity into radians.
        angularVelocity *= (2.0 * Math.PI);

        return angularVelocity;
    }

    /**
     * Gets the steer encoder offset of the swerve module
     * steer encoder, assuming its position will be at
     * zero radians.
     * 
     * @return The steer encoder offset of the swerve module
     * steer encoder.
     */
    public double getSteerEncoderOffset() {
        // Get the angle reading from the swerve module steer encoder in rotations.
        double angle = steerEncoder.getAbsolutePosition().getValueAsDouble();

        // If the swerve module steer encoder is reversed, then flip the angle.
        if (steerEncoderReversed) {
            angle = 1.0 - angle;
        }
        
        // Convert the angle into radians.
        angle *= (2.0 * Math.PI);

        return angle;
    }

    public PIDController getDrivePIDController() {
        return drivePIDController;
    }
    /**
     * Gets the swerve module steer PID controller.
     * 
     * @return The swerve module steer PID controller.
     */
    public ProfiledPIDController getSteerPIDController() {
        return steerPIDController;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveMotorLinearVelocity(), new Rotation2d(getSteerEncoderAngle()));
    }

    /**
     * Gets the current swerve module position.
     * 
     * @return The current swerve module position.
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(getSteerEncoderAngle()));
    }

    /**
     * Sets the swerve module to run at the given 
     * swerve module state.
     * 
     * @param state The swerve module state to run
     * the swerve module at.
     */
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getSteerEncoderAngle()));

        double speed = Math.abs(state.speedMetersPerSecond) <= SwerveModuleConstants.DRIVE_MOTOR_VELOCITY_DEADBAND ? 0.0 : state.speedMetersPerSecond;

        driveMotor.setVoltage(
            driveFeedforward.calculate(speed) 
            + drivePIDController.calculate(driveEncoder.getVelocity(), speed)
        );

        double angle = state.angle.getRadians() < 0.0 ? state.angle.getRadians() + (2.0 * Math.PI) : state.angle.getRadians();

        steerMotor.setVoltage(
            steerPIDController.calculate(getSteerEncoderAngle(), angle)
        );
    }

    /**
     * Sets the swerve module drive motor to the given voltage.
     * 
     * @param voltage The voltage to set the swerve module
     * drive motor to.
     */
    public void setDriveMotorVoltage(Measure<Voltage> voltage) {
        driveMotor.setVoltage(voltage.magnitude());
    }

    /**
     * Sets the swerve module steer motor to the given voltage.
     * 
     * @param voltage The voltage to set the swerve module
     * steer motor to.
     */
    public void setSteerMotorVoltage(Measure<Voltage> voltage) {
        steerMotor.setVoltage(voltage.magnitude());
    }
}
