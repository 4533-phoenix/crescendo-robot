package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public final class Shooter extends SubsystemBase {
    private static Shooter shooter = null;

    /**
     * The left shooter motor.
     * This motor also acts as the lift motor for
     * the shooter subsystem.
     */
    private final CANSparkMax leftShooterMotor = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax rightShooterMotor = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput shooterLimitSwitch = new DigitalInput(ShooterConstants.SHOOTER_LIMIT_SWITCH_ID);

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }

        return shooter;
    }

    private Shooter() {
        leftShooterMotor.setInverted(true);
        rightShooterMotor.setInverted(true);
    }

    public void runShooterForwards() {
        leftShooterMotor.setVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        rightShooterMotor.setVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
    }

    public void runShooterBackwards() {
        leftShooterMotor.setVoltage(-ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        rightShooterMotor.setVoltage(-ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
    }

    public void stopShooter() {
        leftShooterMotor.setVoltage(0.0);
        rightShooterMotor.setVoltage(0.0);
    }

    public void runLiftForwards() {
        leftShooterMotor.setVoltage(ShooterConstants.LIFT_MOTOR_VOLTAGE);
    }

    public void runLiftBackwards() {
        leftShooterMotor.setVoltage(-ShooterConstants.LIFT_MOTOR_VOLTAGE);
    }

    public void stopLift() {
        leftShooterMotor.setVoltage(0.0);
    }

    public boolean isLimitSwitchPressed() {
        return shooterLimitSwitch.get();
    }
}
