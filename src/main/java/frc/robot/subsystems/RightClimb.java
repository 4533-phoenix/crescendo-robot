package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * The class for the right climb subsystem.
 */
public class RightClimb extends SubsystemBase {
    /**
     * The instance of the {@link RightClimb} class.
     */
    private static RightClimb rightClimb = null;

    /**
     * The right climb motor.
     */
    private final CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, MotorType.kBrushed);

    /**
     * The right climb limit switch.
     */
    private final DigitalInput rightClimbLimitSwitch = new DigitalInput(ClimbConstants.RIGHT_CLIMB_LIMIT_SWITCH_ID);

    /**
     * The current of the right climb motor, in amps.
     */
    private double rightClimbCurrent = rightClimbMotor.getOutputCurrent();

    /**
     * Gets the instance of the {@link RightClimb} class.
     * 
     * @return The instance of the {@link RightClimb} class.
     */
    public static RightClimb getInstance() {
        if (rightClimb == null) {
            rightClimb = new RightClimb();
        }

        return rightClimb;
    }

    /**
     * The constructor for the {@link RightClimb} class.
     */
    private RightClimb() {}

    /**
     * Runs the right climb motor up.
     */
    public void runRightClimbUp() {
        rightClimbMotor.setVoltage(ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Runs the right climb motor down.
     */
    public void runRightClimbDown() {
        rightClimbMotor.setVoltage(-ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Stops the right climb motor.
     */
    public void stopRightClimb() {
        rightClimbMotor.setVoltage(0.0);
    }

    /**
     * Gets whether or not the right climb limit switch
     * is at a magnet.
     * 
     * @return Whether or not the right climb limit switch
     * is at a magnet.
     */
    public boolean isRightClimbLimitSwitchAtMagnet() {
        return !rightClimbLimitSwitch.get();
    }

    /**
     * The periodic method for the right climb subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        rightClimbCurrent = rightClimbMotor.getOutputCurrent();
    }
}
