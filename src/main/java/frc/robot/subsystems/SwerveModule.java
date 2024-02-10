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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
     * The swerve module steer feedforward.
     */
    private SimpleMotorFeedforward steerFeedforward;

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
     * offset, and whether or not the swerve module steer 
     * encoder is reversed.
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
     */
    public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed, 
        int steerEncoderID, double steerEncoderOffset, boolean steerEncoderReversed
    ) {
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

        // Create the swerve module drive motor feedforward.
        driveFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.DRIVE_MOTOR_KS, 
            SwerveModuleConstants.DRIVE_MOTOR_KV
        );

        // Create the swerve module steer motor feedforward.
        steerFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.STEER_MOTOR_KS, 
            SwerveModuleConstants.STEER_MOTOR_KV
        );
        
        // Create the swerve module drive motor PID controller.
        drivePIDController = new PIDController(
            SwerveModuleConstants.DRIVE_MOTOR_KP, 
            SwerveModuleConstants.DRIVE_MOTOR_KI, 
            SwerveModuleConstants.DRIVE_MOTOR_KD
        );

        // Create the swerve module steer motor PID controller.
        steerPIDController = new ProfiledPIDController(
            SwerveModuleConstants.STEER_MOTOR_KP, 
            SwerveModuleConstants.STEER_MOTOR_KI, 
            SwerveModuleConstants.STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION
            )
        );

        /*
         * Set continuous input for the swerve module 
         * steer PID controller. 
         */
        steerPIDController.enableContinuousInput(0.0, (2 * Math.PI));
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
        double angle = steerEncoder.getAbsolutePosition().getValueAsDouble();

        if (steerEncoderReversed) {
            angle = 1.0 - angle;
        }
        
        angle *= (2 * Math.PI);
        angle -= steerEncoderOffset;

        if (angle < 0) {
            angle += (2 * Math.PI);
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
        double angularVelocity = steerEncoder.getVelocity().getValueAsDouble();

        angularVelocity *= steerEncoderReversed ? -1.0 : 1.0;
        angularVelocity *= (2 * Math.PI);

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
        double angle = steerEncoder.getAbsolutePosition().getValueAsDouble();

        if (steerEncoderReversed) {
            angle = 1.0 - angle;
        }
        
        angle *= (2 * Math.PI);

        return angle;
    }

    /**
     * Gets the swerve module steer PID controller.
     * 
     * @return The swerve module steer PID controller.
     */
    public ProfiledPIDController getSteerPIDController() {
        return steerPIDController;
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
        // SwerveModuleState.optimize(state, Rotation2d.fromRadians(getSteerEncoderAngle()));

        driveMotor.setVoltage(
            driveFeedforward.calculate(state.speedMetersPerSecond) 
            + drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond)
        );

        double steerPIDValue = steerPIDController.calculate(getSteerEncoderAngle(), state.angle.getRadians());

        steerMotor.setVoltage(
            steerFeedforward.calculate(steerPIDController.getSetpoint().velocity)
            + steerPIDValue
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
