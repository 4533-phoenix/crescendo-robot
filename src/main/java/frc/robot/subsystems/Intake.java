package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The class for the intake subsystem.
 */
public final class Intake extends SubsystemBase {
    private static Intake intake = null;

    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }

        return intake;
    }

    private Intake() {}

    public void runIntakeForward() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_MOTOR_VOLTAGE);
    }

    public void runIntakeBackward() {
        intakeMotor.setVoltage(-IntakeConstants.INTAKE_MOTOR_VOLTAGE);
    }

    public void stopIntake() {
        intakeMotor.setVoltage(0.0);
    }
}
