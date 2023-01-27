// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;
  private DriveSubsystem driveSubsystem=RobotContainer.driveSubsystem;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //m_robotContainer.driveSubsystem.setModePercentVoltage();
    RobotContainer.driveSubsystem.resetEncoders();
  

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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
private double startTime;
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Timer.getFPGATimestamp();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  public static final int Left_front_drive_port=3;
  public static final int Right_front_drive_port=4;
  public static final int Left_back_drive_port=1;
  public static final int Right_back_drive_port=2;
  public static WPI_TalonFX leftFrontDrivePort= new WPI_TalonFX(Left_front_drive_port);
  public static WPI_TalonFX rightFrontDrivePort= new WPI_TalonFX(Right_front_drive_port);
  public static WPI_TalonFX leftBackDrivePort= new WPI_TalonFX(Left_back_drive_port);
  public static WPI_TalonFX rightBackDrivePort= new WPI_TalonFX(Right_back_drive_port);
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
double time = Timer.getFPGATimestamp();
if (time<1){
  leftFrontDrivePort.set(0.1);
  rightFrontDrivePort.set(0.1);
  leftBackDrivePort.set(0.1);
  rightBackDrivePort.set(0.1);
}
else {
  leftFrontDrivePort.set(0);
  rightFrontDrivePort.set(0);
  leftBackDrivePort.set(0);
  rightBackDrivePort.set(0);
}


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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