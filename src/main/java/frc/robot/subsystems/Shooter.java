package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * The class for the shooter subsystem.
 */
public final class Shooter extends SubsystemBase {
    /**
     * The instance of the {@link Shooter} class.
     */
    private static Shooter shooter = null;

    /**
     * The lift motor.
     */
    private final CANSparkMax liftMotor = 
        new CANSparkMax(ShooterConstants.LIFT_MOTOR_ID, MotorType.kBrushless);

    /**
     * The left shooter motor.
     */
    private final CANSparkMax leftShooterMotor = 
        new CANSparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    /**
     * The right shooter motor.
     */
    private final CANSparkMax rightShooterMotor = 
        new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    /**
     * The shooter limit switch.
     */
    private final DigitalInput shooterLimitSwitch = 
        new DigitalInput(ShooterConstants.SHOOTER_LIMIT_SWITCH_ID);

    /**
     * Gets the instance of the {@link Shooter} class.
     * 
     * @return The instance of the {@link Shooter} class.
     */
    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }

        return shooter;
    }

    /**
     * The constructor for the {@link Shooter} class.
     */
    private Shooter() {
        rightShooterMotor.setInverted(true);
    }

    /**
     * Runs the lift motor forwards.
     */
    public void runLiftForwards() {
        liftMotor.setVoltage(ShooterConstants.LIFT_MOTOR_VOLTAGE);
    }

    /**
     * Runs the lift motor backwards.
     */
    public void runLiftBackwards() {
        liftMotor.setVoltage(-ShooterConstants.LIFT_MOTOR_VOLTAGE);
    }

    /**
     * Stops the lift motor.
     */
    public void stopLift() {
        liftMotor.setVoltage(0.0);
    }

    /**
     * Runs the left shooter motor and the
     * right shooter motor forwards.
     */
    public void runShooterForwards() {
        leftShooterMotor.setVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        rightShooterMotor.setVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
    }

    /**
     * Runs the left shooter motor and the
     * right shooter motor backwards.
     */
    public void runShooterBackwards() {
        leftShooterMotor.setVoltage(-ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        rightShooterMotor.setVoltage(-ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
    }

    /**
     * Stops the left shooter motor and the
     * right shooter motor.
     */
    public void stopShooter() {
        leftShooterMotor.setVoltage(0.0);
        rightShooterMotor.setVoltage(0.0);
    }
    
    /**
     * Gets whether or not the shooter limit
     * switch is being pressed.
     * 
     * @return Whether or not the shooter limit
     * switch is being pressed.
     */
    public boolean isLimitSwitchPressed() {
        return shooterLimitSwitch.get();
    }

    /**
     * Gets whether or not the shooter is ready to shoot,
     * which is determined by if the slower shooter motor
     * is above the required shooter velocity.
     * 
     * @return Whether or not the shooter is ready to shoot.
     */
    public boolean isShooterReady() {
        return Math.min(
                leftShooterMotor.getEncoder().getVelocity(),
                rightShooterMotor.getEncoder().getVelocity()) 
            > ShooterConstants.SHOOTER_VELOCITY;
    }

    /**
     * The periodic method for the shooter subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        /*
         * Send whether or not the limit switch is being pressed
         * to Shuffleboard.
         */
        SmartDashboard.putBoolean("Lift Limit Switch", isLimitSwitchPressed());
    }
}
