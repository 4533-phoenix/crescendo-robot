package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * The class for the climb subsystem.
 */
public final class Climb extends SubsystemBase {
    /**
     * The instance of the {@link Climb} class.
     */
    private static Climb climb = null;

    /**
     * The left climb motor.
     */
    private final CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_ID, MotorType.kBrushed);

    /**
     * The right climb motor.
     */
    private final CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, MotorType.kBrushed);

    /**
     * Gets the instance of the {@link Climb} class.
     * 
     * @return The instance of the {@link Climb} class.
     */
    public static Climb getInstance() {
        if (climb == null) {
            climb = new Climb();
        }

        return climb;
    }

    /**
     * The constructor for the {@link Climb} class.
     */
    private Climb() {}

    /**
     * Runs the climb motors up.
     */
    public void runClimbUp() {
        leftClimbMotor.setVoltage(ClimbConstants.CLIMB_MOTOR_VOLTAGE);
        rightClimbMotor.setVoltage(ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Runs the climb motors down.
     */
    public void runClimbDown() {
        leftClimbMotor.setVoltage(-ClimbConstants.CLIMB_MOTOR_VOLTAGE);
        rightClimbMotor.setVoltage(-ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Stops the climb motors.
     */
    public void stopClimb() {
        leftClimbMotor.setVoltage(0.0);
        rightClimbMotor.setVoltage(0.0);
    }
}