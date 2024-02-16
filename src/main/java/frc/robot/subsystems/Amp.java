package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;

/**
 * The class for the amp subsystem.
 */
public final class Amp extends SubsystemBase {
    /**
     * The instance of the {@link Amp} class.
     */
    private static Amp amp = null;

    /**
     * The motor that rotates the amp subsystem.
     */
    // private final CANSparkMax ampMotor = new CANSparkMax(AmpConstants.AMP_MOTOR_ID, MotorType.kBrushless);

    /**
     * The limit switch at the amp subsystem receive position.
     */
    // private final DigitalInput receiveLimitSwitch = new DigitalInput(AmpConstants.RECEIVE_LIMIT_SWITCH_ID);

    /**
     * The limit switch at the amp subsystem drop position.
     */
    // private final DigitalInput dropLimitSwitch = new DigitalInput(AmpConstants.DROP_LIMIT_SWITCH_ID);

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
     * Runs the amp motor forwards, towards the
     * drop position of the amp subsystem.
     */
    public void runAmpMotorForwards() {
        // ampMotor.setVoltage(AmpConstants.AMP_MOTOR_VOLTAGE);
    }

    /**
     * Runs the amp motor backwards, towards the
     * receive position of the amp subsystem.
     */
    public void runAmpMotorBackwards() {
        // ampMotor.setVoltage(-AmpConstants.AMP_MOTOR_VOLTAGE);
    }

    /**
     * Stops the amp motor.
     */
    public void stopAmpMotor() {
        // ampMotor.setVoltage(0.0);
    }

    /**
     * Gets whether or not the receive limit switch
     * is pressed.
     * 
     * @return Whether or not the receive limit switch
     * is pressed.
     */
    public boolean getReceiveLimitSwitchPressed() {
        // return receiveLimitSwitch.get();
        return true;
    }

    /**
     * Gets whether or not the receive limit switch
     * is pressed.
     * 
     * @return Whether or not the receive limit switch
     * is pressed.
     */
    public boolean getDropLimitSwitchPressed() {
        // return dropLimitSwitch.get();
        return true;
    }
}
