// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Supplier<Command> autoSelected = () -> { return new InstantCommand(); };

  private Command autoCommand = new InstantCommand();
  
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<Supplier<Command>>();

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
    
    autoChooser.setDefaultOption(
      AutoConstants.SOURCE_EXIT_AUTO_KEY, 
      () -> AutoCommands.followPathAuto(AutoConstants.SOURCE_EXIT_AUTO_PATH_FILE_NAME)
    );

    autoChooser.addOption(
      AutoConstants.AMP_EXIT_AUTO_KEY, 
      () -> AutoCommands.followPathAuto(AutoConstants.AMP_EXIT_AUTO_PATH_FILE_NAME)
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    field.setRobotPose(Swerve.getInstance().getRobotPose());

    SmartDashboard.putData("Field", field);

    // SmartDashboard.putNumber("Swerve Module ID", 0);

    // SmartDashboard.putBoolean("PID Test", false);

    // int swerveModuleID = (int) SmartDashboard.getNumber("Swerve Module ID", 0.0);

    // SmartDashboard.putNumber("Current Velocity", Swerve.getInstance().getSwerveModule(swerveModuleID).getDriveMotorLinearVelocity());
    // SmartDashboard.putNumber("Velocity Setpoint", 0.0);

    // SmartDashboard.putNumber("Kp", Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().getP());
    // SmartDashboard.putNumber("Ki", Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().getI());
    // SmartDashboard.putNumber("Kd", Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().getD());
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

    autoCommand = autoSelected.get();

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

    //   double Kp = SmartDashboard.getNumber("Kp", Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().getP());
    //   double Ki = SmartDashboard.getNumber("Ki", Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().getI());
    //   double Kd = SmartDashboard.getNumber("Kd", Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().getD());

    //   Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().setP(Kp);
    //   Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().setI(Ki);
    //   Swerve.getInstance().getSwerveModule(swerveModuleID).getDrivePIDController().setD(Kd);

    //   SwerveModuleState swerveModuleState = new SwerveModuleState(SmartDashboard.getNumber("Velocity Setpoint", 0.0), new Rotation2d());

    //   Swerve.getInstance().getSwerveModule(swerveModuleID).setState(swerveModuleState);

    //   SmartDashboard.putNumber("Current Velocity", Swerve.getInstance().getSwerveModule(swerveModuleID).getDriveMotorLinearVelocity());
    // }

    // for (SwerveModule swerveModule : Swerve.getInstance().getSwerveModules()) {
    //   swerveModule.setState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
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
