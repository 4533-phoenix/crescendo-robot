package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * The class for the left climb subsystem.
 */
public class LeftClimb extends SubsystemBase {
    /**
     * The instance of the {@link LeftClimb} class.
     */
    private static LeftClimb leftClimb = null;

    /**
     * The left climb motor.
     */
    private final CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_ID, MotorType.kBrushed);

    /**
     * The left climb limit switch.
     */
    private final DigitalInput leftClimbLimitSwitch = new DigitalInput(ClimbConstants.LEFT_CLIMB_LIMIT_SWITCH_ID);

    /**
     * Gets the instance of the {@link LeftClimb} class.
     * 
     * @return The instance of the {@link LeftClimb} class.
     */
    public static LeftClimb getInstance() {
        if (leftClimb == null) {
            leftClimb = new LeftClimb();
        }

        return leftClimb;
    }

    /**
     * The constructor for the {@link LeftClimb} class.
     */
    private LeftClimb() {}

    /**
     * Runs the left climb motor up.
     */
    public void runLeftClimbUp() {
        leftClimbMotor.setVoltage(ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Runs the left climb motor down.
     */
    public void runLeftClimbDown() {
        leftClimbMotor.setVoltage(-ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Stops the left climb motor.
     */
    public void stopLeftClimb() {
        leftClimbMotor.setVoltage(0.0);
    }

    /**
     * Gets whether or not the left climb limit switch
     * is at a magnet.
     * 
     * @return Whether or not the left climb limit switch
     * is at a magnet.
     */
    public boolean isLeftClimbLimitSwitchAtMagnet() {
        return !leftClimbLimitSwitch.get();
    }
}
