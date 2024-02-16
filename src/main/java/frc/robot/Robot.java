// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String autoSelected = "Tutorial Auto";

  private Command autoCommand = new InstantCommand();

  private Pose2d autoPosition = new Pose2d();
  
  private final SendableChooser<String> autoChooser = new SendableChooser<String>();

  public static final SysIdRoutine sysIDDriveRoutine = new SysIdRoutine(
    new Config(), 
    new Mechanism(
      Swerve.getInstance()::sysIDDriveTest, 
      Swerve.getInstance()::sysIDDriveLog, 
      Swerve.getInstance(), 
      "Test Drive Motors"
    )
  );

  public static final SysIdRoutine sysIDSteerRoutine = new SysIdRoutine(
    new Config(), 
    new Mechanism(
      Swerve.getInstance()::sysIDSteerTest, 
      Swerve.getInstance()::sysIDSteerLog, 
      Swerve.getInstance(), 
      "Test Steer Motors"
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
    
    autoChooser.setDefaultOption("Tutorial Auto", "Tutorial Auto");

    SmartDashboard.putData(autoChooser);

    // SmartDashboard.putNumber("Swerve Module ID", 0);

    // SmartDashboard.putBoolean("PID Test", false);

    // int swerveModuleID = (int) SmartDashboard.getNumber("Swerve Module ID", 0.0);

    // SmartDashboard.putNumber("Current Angle", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerEncoderAngle() * (180.0 / Math.PI));
    // SmartDashboard.putNumber("Angle Setpoint", 0.0);

    // SmartDashboard.putNumber("Kp", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().getP());
    // SmartDashboard.putNumber("Ki", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().getI());
    // SmartDashboard.putNumber("Kd", Swerve.getInstance().getSwerveModule(swerveModuleID).getSteerPIDController().getD());
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
    autoSelected = autoChooser.getSelected();

    autoCommand = RobotContainer.getAutonomous(autoSelected);

    autoPosition = RobotContainer.getAutonomousPosition(autoSelected);

    Swerve.getInstance().registerPoseEstimator(autoPosition);

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
    // if (SmartDashboard.getBoolean("PID Test", false)) {
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
