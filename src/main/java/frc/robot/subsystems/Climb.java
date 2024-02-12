package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public final class Climb extends SubsystemBase {
    private static Climb climb = null;

    private final CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.CLIMB_LEFT_MOTOR_ID, MotorType.kBrushed);
    private final CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushed);

    public static Climb getInstance() {
        if (climb == null) {
            climb = new Climb();
        }

        return climb;
    }

    private Climb() {
    }

    public void runClimbUp() {
        System.out.println("up");
        leftClimbMotor.setVoltage(ClimbConstants.CLIMB_LEFT_VOLTAGE);
        rightClimbMotor.setVoltage(ClimbConstants.CLIMB_RIGHT_VOLTAGE);
    }

    public void runClimbDown() {
        System.out.println("down");
        leftClimbMotor.setVoltage(ClimbConstants.CLIMB_LEFT_VOLTAGE);
        rightClimbMotor.setVoltage(ClimbConstants.CLIMB_RIGHT_VOLTAGE);
    }

    public void stopClimb() {
        System.out.println("stop");
        leftClimbMotor.setVoltage(ClimbConstants.CLIMB_LEFT_VOLTAGE);
        rightClimbMotor.setVoltage(ClimbConstants.CLIMB_RIGHT_VOLTAGE);
    }
}