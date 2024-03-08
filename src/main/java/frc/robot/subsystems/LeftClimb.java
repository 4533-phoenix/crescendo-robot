package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MOVEMENT_DIRECTION;
import frc.robot.Constants.ClimbConstants.CLIMB_POSITION;

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
    // private final CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_ID, MotorType.kBrushed);

    /**
     * The left climb limit switch.
     */
    // private final DigitalInput leftClimbLimitSwitch = new DigitalInput(ClimbConstants.LEFT_CLIMB_LIMIT_SWITCH_ID);

    /**
     * The current position of the left climb subsystem.
     * Represents where the left climb subsystem is or 
     * last was.
     */
    private CLIMB_POSITION leftClimbPosition = CLIMB_POSITION.DOWN_POSITION;
    
    /**
     * The current movement direction of the left climb subsystem.
     * Represents which position the left climb subsystem
     * is currently moving to or last was moving to.
     */
    private CLIMB_MOVEMENT_DIRECTION leftClimbMovementDirection = CLIMB_MOVEMENT_DIRECTION.TOWARDS_UP_POSITION;

    /**
     * The previous state of the left climb limit switch.
     */
    private boolean prevLeftClimbLimitSwitchState = false;

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
        // leftClimbMotor.setVoltage(ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Runs the left climb motor down.
     */
    public void runLeftClimbDown() {
        // leftClimbMotor.setVoltage(-ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Stops the left climb motor.
     */
    public void stopLeftClimb() {
        // leftClimbMotor.setVoltage(0.0);
    }

    /**
     * Gets whether or not the left climb limit switch is at
     * the given position.
     * 
     * @param leftClimbPosition The climb position to
     * check whether or not the left climb limit switch
     * is at.
     * 
     * @return Whether or not the left climb limit switch
     * is at the given position.
     */
    public boolean isLeftClimbLimitSwitchAtPosition(CLIMB_POSITION leftClimbPosition) {
        return this.leftClimbPosition == leftClimbPosition;
    }

    /**
     * Gets whether or not the left climb limit switch
     * is at a magnet.
     * 
     * @return Whether or not the left climb limit switch
     * is at a magnet.
     */
    public boolean isLeftClimbLimitSwitchAtMagnet() {
        // return !leftClimbLimitSwitch.get();
        return true;
    }

    /**
     * Sets the left climb subsystem movement direction
     * to the given climb movement direction.
     * 
     * @param leftClimbMovementDirection The climb movement direction 
     * to set the left climb subsystem movement direction to.
     */
    public void setLeftClimbMovementDirection(CLIMB_MOVEMENT_DIRECTION leftClimbMovementDirection) {
        this.leftClimbMovementDirection = leftClimbMovementDirection;
    }

    /**
     * The periodic method for the left climb subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        /*
         * If the left climb limit switch is at a magnet
         * and if the left climb limit switch previously
         * was not at a magnet, then check to see
         * if the left climb subsystem position should 
         * be updated.
         */
        if (isLeftClimbLimitSwitchAtMagnet() && prevLeftClimbLimitSwitchState) {
            /*
             * If the left climb subsystem position is the down position
             * and the left climb subsystem movement direction is towards
             * the up position, then update the left climb subsystem
             * position to be the up position.
             * 
             * If not, and if the left climb subsystem position is the up 
             * position and the left climb subsystem movement direction is towards
             * the down position, then update the left climb subsystem
             * position to be the down position.
             */
            if (leftClimbPosition == CLIMB_POSITION.DOWN_POSITION 
                    && leftClimbMovementDirection == CLIMB_MOVEMENT_DIRECTION.TOWARDS_UP_POSITION) {
                leftClimbPosition = CLIMB_POSITION.UP_POSITION;
            } else if (leftClimbPosition == CLIMB_POSITION.UP_POSITION
                    && leftClimbMovementDirection == CLIMB_MOVEMENT_DIRECTION.TOWARDS_DOWN_POSITION) {
                leftClimbPosition = CLIMB_POSITION.DOWN_POSITION;
            }
        }

        // Update the previous left climb limit switch state.
        // prevLeftClimbLimitSwitchState = leftClimbLimitSwitch.get();
    }
}
