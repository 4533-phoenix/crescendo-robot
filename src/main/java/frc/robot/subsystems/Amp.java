package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.AmpConstants.AMP_MOVEMENT_DIRECTION;
import frc.robot.Constants.AmpConstants.AMP_POSITION;

/**
 * The class for the amp subsystem.
 */
public final class Amp extends SubsystemBase {
    /**
     * The instance of the {@link Amp} class.
     */
    private static Amp amp = null;

    /**
     * The amp motor.
     */
    private final CANSparkMax ampMotor = new CANSparkMax(AmpConstants.AMP_MOTOR_ID, MotorType.kBrushless);

    /**
     * The amp limit switch.
     */
    private final DigitalInput ampLimitSwitch = new DigitalInput(AmpConstants.AMP_LIMIT_SWITCH_ID);

    /**
     * The current position of the amp subsystem.
     * This position can either be where the amp
     * subsystem is or where the amp subsystem
     * last was.
     */
    private AMP_POSITION ampPosition = AMP_POSITION.RECEIVE_POSITION;

    /**
     * The current movement direction of the amp subsystem.
     * Represents which position the amp subsystem
     * is currently moving to or last was moving to.
     */
    private AMP_MOVEMENT_DIRECTION ampMovementDirection = AMP_MOVEMENT_DIRECTION.TOWARDS_DROP_POSITION;

    /**
     * The previous state of the amp limit switch.
     */
    private boolean prevAmpLimitSwitchState = false;

    /**
     * Gets the instance of the {@link Amp} class.
     * 
     * @return The instance of the {@link Amp} class.
     */
    public static Amp getInstance() {
        if (amp == null) {
            amp = new Amp();
        }

        return amp;
    }

    /**
     * The constructor for the {@link Amp} class.
     */
    private Amp() {}

    /**
     * Runs the amp motor forwards.
     */
    public void runAmpForwards() {
        ampMotor.setVoltage(AmpConstants.AMP_MOTOR_VOLTAGE);
    }

    /**
     * Runs the amp motor backwards.
     */
    public void runAmpBackwards() {
        ampMotor.setVoltage(-AmpConstants.AMP_MOTOR_VOLTAGE);
    }

    /**
     * Stops the amp motor.
     */
    public void stopAmp() {
        ampMotor.setVoltage(0.0);
    }

    /**
     * Gets whether or not the amp limit switch
     * is at the given position.
     * 
     * @return Whether or not the amp limit switch
     * is at the given position.
     */
    public boolean isLimitSwitchAtPosition(AMP_POSITION ampPosition) {
        return this.ampPosition == ampPosition;
    }

    /**
     * Gets whether or not the amp limit switch
     * is at a magnet.
     * 
     * @return Whether or not the amp limit switch
     * is at a magnet.
     */
    public boolean isLimitSwitchAtMagnet() {
        return !ampLimitSwitch.get();
    }

    /**
     * Sets the amp subsystem movement direction to the
     * given movement direction.
     * 
     * @param ampMovementDirection The movement direction
     * to set the amp subsystem movement direction to.
     */
    public void setAmpMovementDirection(AMP_MOVEMENT_DIRECTION ampMovementDirection) {
        this.ampMovementDirection = ampMovementDirection;
    }

    /**
     * The periodic method for the amp subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        /*
         * If the amp limit switch is at a magnet
         * and if the amp limit switch previously
         * was not at a magnet, then check to see
         * if the amp subsystem position should 
         * be updated.
         */
        if (!ampLimitSwitch.get() && prevAmpLimitSwitchState) {
            /*
             * If the amp subsystem position is the receive position
             * and the amp subsystem movement direction is towards
             * the drop position, then update the amp subsystem
             * position to be the drop position.
             * 
             * If not, and if the amp subsystem position is the drop 
             * position and the amp subsystem movement direction is towards
             * the receive position, then update the amp subsystem
             * position to be the receive position.
             */
            if (ampPosition == AMP_POSITION.RECEIVE_POSITION && ampMovementDirection == AMP_MOVEMENT_DIRECTION.TOWARDS_DROP_POSITION) {
                ampPosition = AMP_POSITION.DROP_POSITION;
            } else if (ampPosition == AMP_POSITION.DROP_POSITION && ampMovementDirection == AMP_MOVEMENT_DIRECTION.TOWARDS_RECEIVE_POSITION) {
                ampPosition = AMP_POSITION.RECEIVE_POSITION;
            }
        }

        /*
         * Update the previous amp limit switch state.
         */
        prevAmpLimitSwitchState = ampLimitSwitch.get();
    }
}