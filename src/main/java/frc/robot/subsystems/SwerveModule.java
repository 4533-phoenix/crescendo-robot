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
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Constants.SwerveModuleConstants;

public final class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;

    private RelativeEncoder driveEncoder;
    private CANcoder steerEncoder;

    private double steerEncoderOffset;
    private boolean steerEncoderReversed;

    private SimpleMotorFeedforward driveFeedforward;
    private SimpleMotorFeedforward steerFeedforward;

    private PIDController drivePIDController;
    private ProfiledPIDController steerPIDController;

    public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed, 
        int steerEncoderID, double steerEncoderOffset, boolean steerEncoderReversed
    ) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        
        driveEncoder = driveMotor.getEncoder();

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_REVOLUTIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_RPM_TO_METERS_PER_SECOND);
        
        steerEncoder = new CANcoder(steerEncoderID, "rio");

        steerEncoder.getConfigurator().apply(
            new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            )
        );

        this.steerEncoderOffset = steerEncoderOffset;
        this.steerEncoderReversed = steerEncoderReversed;

        driveFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.DRIVE_MOTOR_KS, 
            SwerveModuleConstants.DRIVE_MOTOR_KV
        );
        steerFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.STEER_MOTOR_KS, 
            SwerveModuleConstants.STEER_MOTOR_KV
        );
        
        drivePIDController = new PIDController(
            SwerveModuleConstants.DRIVE_MOTOR_KP, 
            SwerveModuleConstants.DRIVE_MOTOR_KI, 
            SwerveModuleConstants.DRIVE_MOTOR_KD
        );
        steerPIDController = new ProfiledPIDController(
            SwerveModuleConstants.STEER_MOTOR_KP, 
            SwerveModuleConstants.STEER_MOTOR_KI, 
            SwerveModuleConstants.STEER_MOTOR_KD, 
            new Constraints(
                SwerveModuleConstants.STEER_MOTOR_MAX_VELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_MAX_ACCELERATION
            )
        );

        steerPIDController.enableContinuousInput(0.0, (2 * Math.PI));
    }

    public double getDriveMotorVoltage() {
        return driveMotor.get() * RobotController.getBatteryVoltage();
    }

    public double getSteerMotorVoltage() {
        return steerMotor.get() * RobotController.getBatteryVoltage();
    }

    public double getDriveMotorLinearPosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveMotorLinearVelocity() {
        return driveEncoder.getVelocity();
    }

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

    public double getSteerEncoderAngularVelocity() {
        double angularVelocity = steerEncoder.getVelocity().getValueAsDouble();

        angularVelocity *= steerEncoderReversed ? -1.0 : 1.0;
        angularVelocity *= (2 * Math.PI);

        return angularVelocity;
    }

    public double getSteerEncoderOffset() {
        double angle = steerEncoder.getAbsolutePosition().getValueAsDouble();

        if (steerEncoderReversed) {
            angle = 1.0 - angle;
        }
        
        angle *= (2 * Math.PI);

        return angle;
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(getSteerEncoderAngle()));
    }

    public void setState(SwerveModuleState state) {
        // SwerveModuleState.optimize(state, Rotation2d.fromRadians(getSteerEncoderValue()));

        // driveMotor.setVoltage(
        //     driveFeedforward.calculate(state.speedMetersPerSecond) 
        //     + drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond)
        // );

        // double steerPIDValue = steerPIDController.calculate(getSteerEncoderValue(), state.angle.getRadians());

        // steerMotor.setVoltage(
        //     steerFeedforward.calculate(steerPIDController.getSetpoint().velocity)
        //     + steerPIDValue
        // );

        steerMotor.setVoltage(5.0);
    }

    public void setDriveMotorVoltage(Measure<Voltage> voltage) {
        driveMotor.setVoltage(voltage.magnitude());
    }

    public void setSteerMotorVoltage(Measure<Voltage> voltage) {
        steerMotor.setVoltage(voltage.magnitude());
    }
}
