// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.helpers.LimelightHelper;

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

    autoChooser.addOption(
      "Quadruple Speaker Score Auto",
      AutoCommands.getQuadrupleSpeakerScoreAuto());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    field.setRobotPose(Swerve.getInstance().getRobotPose());

    SmartDashboard.putData("Field", field);

    // Auto PID test code.

    // SmartDashboard.putBoolean("Auto PID Test", false);

    // HolonomicDriveController holonomicDriveController =
    //   Swerve.getInstance().getHolonomicDriveController();

    // SmartDashboard.putNumber(
    //   "X Kp", 
    //   holonomicDriveController.getXController().getP());
    // SmartDashboard.putNumber(
    //   "X Ki", 
    //   holonomicDriveController.getXController().getI());
    // SmartDashboard.putNumber(
    //   "X Kd", 
    //   holonomicDriveController.getXController().getD());

    // SmartDashboard.putNumber(
    //   "Y Kp", 
    //   holonomicDriveController.getYController().getP());
    // SmartDashboard.putNumber(
    //   "Y Ki",
    //   holonomicDriveController.getYController().getI());
    // SmartDashboard.putNumber(
    //   "Y Kd", 
    //   holonomicDriveController.getYController().getD());
    
    // SmartDashboard.putNumber(
    //   "Theta Kp", 
    //   holonomicDriveController.getThetaController().getP());
    // SmartDashboard.putNumber(
    //   "Theta Ki", 
    //   holonomicDriveController.getThetaController().getI());
    // SmartDashboard.putNumber(
    //   "Theta Kd", 
    //   holonomicDriveController.getThetaController().getD());

    // Drive PID test code.

    // SmartDashboard.putBoolean("Drive PID Test", false);

    // SmartDashboard.putNumber("Swerve Module ID", 0.0);

    // SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(0);

    // SmartDashboard.putNumber(
    //   "Current Velocity", 
    //   swerveModule.getDriveMotorLinearVelocity());
    
    // SmartDashboard.putNumber("Velocity Setpoint", 0.0);

    // PIDController drivePIDController =
    //   swerveModule.getDrivePIDController();

    // SmartDashboard.putNumber("kP", drivePIDController.getP());
    // SmartDashboard.putNumber("kI", drivePIDController.getI());
    // SmartDashboard.putNumber("kD", drivePIDController.getD());

    // Steer PID test code.

    // SmartDashboard.putBoolean("Steer PID Test", false);

    // SmartDashboard.putNumber("Swerve Module ID", 0.0);

    // SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(0);

    // SmartDashboard.putNumber(
    //   "Current Angle",
    //   swerveModule.getSteerEncoderAngle() * (180.0 / Math.PI));
    
    // SmartDashboard.putNumber("Angle Setpoint", 0.0);

    // ProfiledPIDController steerPIDController =
    //   swerveModule.getSteerPIDController();

    // SmartDashboard.putNumber("kP", steerPIDController.getP());
    // SmartDashboard.putNumber("kI", steerPIDController.getI());
    // SmartDashboard.putNumber("kD", steerPIDController.getD());
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

    // Auto PID test code.

    // HolonomicDriveController holonomicDriveController =
    //   Swerve.getInstance().getHolonomicDriveController();

    // if (SmartDashboard.getBoolean("Auto PID Test", false)) {
    //   double xKp = SmartDashboard.getNumber(
    //     "X Kp", 
    //     holonomicDriveController.getXController().getP());
    //   double xKi = SmartDashboard.getNumber(
    //     "X Ki", 
    //     holonomicDriveController.getXController().getI());
    //   double xKd = SmartDashboard.getNumber(
    //     "X Kd", 
    //     holonomicDriveController.getXController().getD());

    //   double yKp = SmartDashboard.getNumber(
    //     "Y Kp", 
    //     holonomicDriveController.getYController().getP());
    //   double yKi = SmartDashboard.getNumber(
    //     "Y Ki", 
    //     holonomicDriveController.getYController().getI());
    //   double yKd = SmartDashboard.getNumber(
    //     "Y Kd", 
    //     holonomicDriveController.getYController().getD());
      
    //   double thetaKp = SmartDashboard.getNumber(
    //     "Theta Kp", 
    //     holonomicDriveController.getThetaController().getP());
    //   double thetaKi = SmartDashboard.getNumber(
    //     "Theta Ki", 
    //     holonomicDriveController.getThetaController().getI());
    //   double thetaKd = SmartDashboard.getNumber(
    //     "Theta Kd", 
    //     holonomicDriveController.getThetaController().getD());

    //   holonomicDriveController.getXController().setP(xKp);
    //   holonomicDriveController.getXController().setI(xKi);
    //   holonomicDriveController.getXController().setD(xKd);

    //   holonomicDriveController.getYController().setP(yKp);
    //   holonomicDriveController.getYController().setI(yKi);
    //   holonomicDriveController.getYController().setD(yKd);

    //   holonomicDriveController.getThetaController().setP(thetaKp);
    //   holonomicDriveController.getThetaController().setI(thetaKi);
    //   holonomicDriveController.getThetaController().setD(thetaKd);
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
    // Drive PID test code.

    // if (SmartDashboard.getBoolean("Drive PID Test", false)) {
    //   SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(
    //     (int) SmartDashboard.getNumber("Swerve Module ID", 0.0));

    //   PIDController drivePIDController =
    //     swerveModule.getDrivePIDController();

    //   drivePIDController.setP(
    //     SmartDashboard.getNumber(
    //       "kP", drivePIDController.getP()));
      
    //   drivePIDController.setI(
    //     SmartDashboard.getNumber(
    //       "kI", drivePIDController.getI()));

    //   drivePIDController.setD(
    //     SmartDashboard.getNumber(
    //       "kD", drivePIDController.getD()));

    //   swerveModule.setState(
    //     new SwerveModuleState(
    //       SmartDashboard.getNumber("Velocity Setpoint", 0.0), 
    //       new Rotation2d()));
      
    //   SmartDashboard.putNumber(
    //     "Current Velocity", 
    //     swerveModule.getDriveMotorLinearVelocity());
    // }
    
    // Steer PID test code.

    // if (SmartDashboard.getBoolean("Steer PID Test", false)) {
    //   SwerveModule swerveModule = Swerve.getInstance().getSwerveModule(
    //     (int) SmartDashboard.getNumber("Swerve Module ID", 0.0));

    //   ProfiledPIDController steerPIDController =
    //     swerveModule.getSteerPIDController();

    //   steerPIDController.setP(
    //     SmartDashboard.getNumber(
    //       "Kp", steerPIDController.getP()));
      
    //   steerPIDController.setI(
    //     SmartDashboard.getNumber(
    //       "Ki", steerPIDController.getI()));
      
    //   steerPIDController.setD(
    //     SmartDashboard.getNumber(
    //       "Kd", steerPIDController.getD()));

    //   swerveModule.setState(
    //     new SwerveModuleState(
    //       0.0, 
    //       Rotation2d.fromDegrees(
    //         SmartDashboard.getNumber("Angle Setpoint", 0.0))));

    //   SmartDashboard.putNumber(
    //     "Current Angle", 
    //     swerveModule.getSteerEncoderAngle() 
    //       * (180.0 / Math.PI));
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
