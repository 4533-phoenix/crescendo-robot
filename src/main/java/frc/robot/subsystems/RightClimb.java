package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MOVEMENT_DIRECTION;
import frc.robot.Constants.ClimbConstants.CLIMB_POSITION;

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
    // private final CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, MotorType.kBrushed);

    /**
     * The right climb limit switch.
     */
    // private final DigitalInput rightClimbLimitSwitch = new DigitalInput(ClimbConstants.RIGHT_CLIMB_LIMIT_SWITCH_ID);

    /**
     * The current position of the right climb subsystem.
     * Represents where the right climb subsystem is or 
     * last was.
     */
    private CLIMB_POSITION rightClimbPosition = CLIMB_POSITION.DOWN_POSITION;
    
    /**
     * The current movement direction of the right climb subsystem.
     * Represents which position the right climb subsystem
     * is currently moving to or last was moving to.
     */
    private CLIMB_MOVEMENT_DIRECTION rightClimbMovementDirection = CLIMB_MOVEMENT_DIRECTION.TOWARDS_UP_POSITION;

    /**
     * The previous state of the right climb limit switch.
     */
    private boolean prevRightClimbLimitSwitchState = false;

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
        // rightClimbMotor.setVoltage(ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Runs the right climb motor down.
     */
    public void runRightClimbDown() {
        // rightClimbMotor.setVoltage(-ClimbConstants.CLIMB_MOTOR_VOLTAGE);
    }

    /**
     * Stops the right climb motor.
     */
    public void stopRightClimb() {
        // rightClimbMotor.setVoltage(0.0);
    }

    /**
     * Gets whether or not the right climb limit switch is at
     * the given position.
     * 
     * @param rightClimbPosition The climb position to
     * check whether or not the right climb limit switch
     * is at.
     * 
     * @return Whether or not the right climb limit switch
     * is at the given position.
     */
    public boolean isRightClimbLimitSwitchAtPosition(CLIMB_POSITION rightClimbPosition) {
        return this.rightClimbPosition == rightClimbPosition;
    }

    /**
     * Gets whether or not the right climb limit switch
     * is at a magnet.
     * 
     * @return Whether or not the right climb limit switch
     * is at a magnet.
     */
    public boolean isRightClimbLimitSwitchAtMagnet() {
        // return !rightClimbLimitSwitch.get();
        return true;
    }

    /**
     * Sets the right climb subsystem movement direction
     * to the given climb movement direction.
     * 
     * @param rightClimbMovementDirection The climb movement direction 
     * to set the right climb subsystem movement direction to.
     */
    public void setRightClimbMovementDirection(CLIMB_MOVEMENT_DIRECTION rightClimbMovementDirection) {
        this.rightClimbMovementDirection = rightClimbMovementDirection;
    }

    /**
     * The periodic method for the right climb subsystem. This method
     * is run by the command scheduler every 20 ms.
     */
    @Override
    public void periodic() {
        /*
         * If the right climb limit switch is at a magnet
         * and if the right climb limit switch previously
         * was not at a magnet, then check to see
         * if the right climb subsystem position should 
         * be updated.
         */
        if (isRightClimbLimitSwitchAtMagnet() && prevRightClimbLimitSwitchState) {
            /*
             * If the right climb subsystem position is the down position
             * and the right climb subsystem movement direction is towards
             * the up position, then update the right climb subsystem
             * position to be the up position.
             * 
             * If not, and if the right climb subsystem position is the up 
             * position and the right climb subsystem movement direction is towards
             * the down position, then update the right climb subsystem
             * position to be the down position.
             */
            if (rightClimbPosition == CLIMB_POSITION.DOWN_POSITION 
                    && rightClimbMovementDirection == CLIMB_MOVEMENT_DIRECTION.TOWARDS_UP_POSITION) {
                rightClimbPosition = CLIMB_POSITION.UP_POSITION;
            } else if (rightClimbPosition == CLIMB_POSITION.UP_POSITION
                    && rightClimbMovementDirection == CLIMB_MOVEMENT_DIRECTION.TOWARDS_DOWN_POSITION) {
                rightClimbPosition = CLIMB_POSITION.DOWN_POSITION;
            }
        }

        // Update the previous right climb limit switch state.
        // prevRightClimbLimitSwitchState = rightClimbLimitSwitch.get();
    }
}
