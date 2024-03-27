// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ShooterCommands;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCommand = new InstantCommand();
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final Field2d field = new Field2d();

  private final NetworkTable robotTable 
    = NetworkTableInstance.getDefault().getTable("Robot");

  public static final SysIdRoutine sysIDDriveRoutine = new SysIdRoutine(
    new Config(), 
    new Mechanism(
      Swerve.getInstance()::sysIDDriveTest, 
      Swerve.getInstance()::sysIDDriveLog, 
      Swerve.getInstance(), 
      "Test Drive Motors"
    )
  );

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Swerve.getInstance().resetGyro();

    RobotContainer.registerSubsystems();

    RobotContainer.registerButtons();
    
    autoChooser.setDefaultOption(
      AutoConstants.DO_NOTHING_AUTO_KEY, 
      new InstantCommand());
    
    autoChooser.addOption(
      AutoConstants.SOURCE_EXIT_AUTO_KEY, 
      AutoCommands.followPathAuto(AutoConstants.SOURCE_EXIT_AUTO_PATH_FILE_NAME));

    autoChooser.addOption(
      AutoConstants.ONLY_SHOOT_AUTO_KEY,
      ShooterCommands.getShootNoteCommand());

    autoChooser.addOption(
      AutoConstants.SPEAKER_SCORE_AUTO_KEY,
      AutoCommands.getSpeakerScoreAuto());

    autoChooser.addOption(
      AutoConstants.SPEAKER_SCORE_TO_CENTER_AUTO_KEY, 
      AutoCommands.getSpeakerScoreToCenterAuto());

    autoChooser.addOption(
      AutoConstants.DOUBLE_SPEAKER_SCORE_AUTO_KEY,
      AutoCommands.getDoubleSpeakerScoreAuto());

    autoChooser.addOption(
      AutoConstants.TRIPLE_SPEAKER_SCORE_AUTO_KEY, 
      AutoCommands.getTripleSpeakerScoreAuto());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    field.setRobotPose(Swerve.getInstance().getRobotPose());

    SmartDashboard.putData("Field", field);

    // SmartDashboard.putBoolean("Auto PID Test", false);

    // SmartDashboard.putNumber(
    //   "X Kp", 
    //   Swerve.getInstance().getHolonomicDriveController().getXController().getP());
    // SmartDashboard.putNumber(
    //   "X Ki", 
    //   Swerve.getInstance().getHolonomicDriveController().getXController().getI());
    // SmartDashboard.putNumber(
    //   "X Kd", 
    //   Swerve.getInstance().getHolonomicDriveController().getXController().getD());

    // SmartDashboard.putNumber(
    //   "Y Kp", 
    //   Swerve.getInstance().getHolonomicDriveController().getYController().getP());
    // SmartDashboard.putNumber(
    //   "Y Ki",
    //   Swerve.getInstance().getHolonomicDriveController().getYController().getI());
    // SmartDashboard.putNumber(
    //   "Y Kd", 
    //   Swerve.getInstance().getHolonomicDriveController().getYController().getD());
    
    // SmartDashboard.putNumber(
    //   "Theta Kp", 
    //   Swerve.getInstance().getHolonomicDriveController().getThetaController().getP());
    // SmartDashboard.putNumber(
    //   "Theta Ki", 
    //   Swerve.getInstance().getHolonomicDriveController().getThetaController().getI());
    // SmartDashboard.putNumber(
    //   "Theta Kd", 
    //   Swerve.getInstance().getHolonomicDriveController().getThetaController().getD());

    // SmartDashboard.putBoolean("Drive PID Test", false);

    // SmartDashboard.putNumber("Swerve Module ID", 0.0);

    // SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(0);

    // SmartDashboard.putNumber("Current Velocity", swerveModule.getDriveMotorLinearVelocity());
    
    // SmartDashboard.putNumber("Velocity Setpoint", 0.0);

    // SmartDashboard.putNumber("kP", swerveModule.getDrivePIDController().getP());
    // SmartDashboard.putNumber("kI", swerveModule.getDrivePIDController().getI());
    // SmartDashboard.putNumber("kD", swerveModule.getDrivePIDController().getD());

    // SmartDashboard.putBoolean("Steer PID Test", false);

    // SmartDashboard.putNumber("Swerve Module ID", 0.0);

    // SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(0);

    // SmartDashboard.putNumber(
    //   "Current Angle",
    //   swerveModule.getSteerEncoderAngle() * (180.0 / Math.PI));
    
    // SmartDashboard.putNumber("Angle Setpoint", 0.0);

    // SmartDashboard.putNumber("kP", swerveModule.getSteerPIDController().getP());
    // SmartDashboard.putNumber("kI", swerveModule.getSteerPIDController().getI());
    // SmartDashboard.putNumber("kD", swerveModule.getSteerPIDController().getD());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    field.setRobotPose(Swerve.getInstance().getRobotPose());

    robotTable.putValue(
      "time", 
      NetworkTableValue.makeDouble(
        Timer.getFPGATimestamp()));

    // if (SmartDashboard.getBoolean("Auto PID Test", false)) {
    //   double xKp = SmartDashboard.getNumber(
    //     "X Kp", 
    //     Swerve.getInstance().getHolonomicDriveController().getXController().getP());
    //   double xKi = SmartDashboard.getNumber(
    //     "X Ki", 
    //     Swerve.getInstance().getHolonomicDriveController().getXController().getI());
    //   double xKd = SmartDashboard.getNumber(
    //     "X Kd", 
    //     Swerve.getInstance().getHolonomicDriveController().getXController().getD());

    //   double yKp = SmartDashboard.getNumber(
    //     "Y Kp", 
    //     Swerve.getInstance().getHolonomicDriveController().getYController().getP());
    //   double yKi = SmartDashboard.getNumber(
    //     "Y Ki", 
    //     Swerve.getInstance().getHolonomicDriveController().getYController().getI());
    //   double yKd = SmartDashboard.getNumber(
    //     "Y Kd", 
    //     Swerve.getInstance().getHolonomicDriveController().getYController().getD());
      
    //   double thetaKp = SmartDashboard.getNumber(
    //     "Theta Kp", 
    //     Swerve.getInstance().getHolonomicDriveController().getThetaController().getP());
    //   double thetaKi = SmartDashboard.getNumber(
    //     "Theta Ki", 
    //     Swerve.getInstance().getHolonomicDriveController().getThetaController().getI());
    //   double thetaKd = SmartDashboard.getNumber(
    //     "Theta Kd", 
    //     Swerve.getInstance().getHolonomicDriveController().getThetaController().getD());

    //   Swerve.getInstance().getHolonomicDriveController().getXController().setP(xKp);
    //   Swerve.getInstance().getHolonomicDriveController().getXController().setI(xKi);
    //   Swerve.getInstance().getHolonomicDriveController().getXController().setD(xKd);

    //   Swerve.getInstance().getHolonomicDriveController().getYController().setP(yKp);
    //   Swerve.getInstance().getHolonomicDriveController().getYController().setI(yKi);
    //   Swerve.getInstance().getHolonomicDriveController().getYController().setD(yKd);

    //   Swerve.getInstance().getHolonomicDriveController().getThetaController().setP(thetaKp);
    //   Swerve.getInstance().getHolonomicDriveController().getThetaController().setI(thetaKi);
    //   Swerve.getInstance().getHolonomicDriveController().getThetaController().setD(thetaKd);
    // }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoCommand = autoChooser.getSelected();

    CommandScheduler.getInstance().schedule(autoCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (autoCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(autoCommand);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (SmartDashboard.getBoolean("Drive PID Test", false)) {
    //   SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(
    //     (int) SmartDashboard.getNumber("Swerve Module ID", 0.0));

    //   swerveModule.getDrivePIDController().setP(
    //     SmartDashboard.getNumber(
    //       "kP", swerveModule.getDrivePIDController().getP()));
      
    //   swerveModule.getDrivePIDController().setI(
    //     SmartDashboard.getNumber(
    //       "kI", swerveModule.getDrivePIDController().getI()));

    //   swerveModule.getDrivePIDController().setD(
    //     SmartDashboard.getNumber(
    //       "kD", swerveModule.getDrivePIDController().getD()));

    //   swerveModule.setState(
    //     new SwerveModuleState(
    //       SmartDashboard.getNumber("Velocity Setpoint", 0.0), new Rotation2d()));
      
    //   SmartDashboard.putNumber("Current Velocity", swerveModule.getDriveMotorLinearVelocity());
    // }

    // if (SmartDashboard.getBoolean("Steer PID Test", false)) {
    //   int swerveModuleID = (int) SmartDashboard.getNumber("Swerve Module ID", 0.0);

    //   double Kp = SmartDashboard.getNumber("Kp", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().getP());
    //   double Ki = SmartDashboard.getNumber("Ki", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().getI());
    //   double Kd = SmartDashboard.getNumber("Kd", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().getD());

    //   Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().setP(Kp);
    //   Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().setI(Ki);
    //   Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().setD(Kd);

    //   SwerveModuleState swerveModuleState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(SmartDashboard.getNumber("Angle Setpoint", 0.0)));

    //   Swerve.getInstance().getSwerveModule(swerveModuleID).setState(swerveModuleState);

    //   SmartDashboard.putNumber("Current Angle", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerEncoderAngle() * (180.0 / Math.PI));
    // }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {} 

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
