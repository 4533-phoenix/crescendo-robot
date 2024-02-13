package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The class for the intake subsystem.
 */
public final class Intake extends SubsystemBase {
    /**
     * The instance of the {@link Intake} class.
     */
    private static Intake intake = null;

    /**
     * The intake motor.
     */
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    /**
     * Gets the instance of the {@link Intake} class.
     * 
     * @return The instance of the {@link Intake} class.
     */
    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }

        return intake;
    }

    /**
     * The constructor for the {@link Intake} class.
     */
    private Intake() {}

    /**
     * Runs the intake motor forwards.
     */
    public void runIntakeForward() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_MOTOR_VOLTAGE);
    }

    /**
     * Runs the intake motor backwards.
     */
    public void runIntakeBackward() {
        intakeMotor.setVoltage(-IntakeConstants.INTAKE_MOTOR_VOLTAGE);
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMotor.setVoltage(0.0);
    }
}
