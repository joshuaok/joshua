// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final static XboxController driverController=RobotContainer.driverController;
  public JoystickDrive(DriveSubsystem drivetrain) {
    driveSubsystem = drivetrain;
    addRequirements(driveSubsystem);
    
  }

  @Override
  public void initialize() {
    driveSubsystem.stop();
  }

  @Override
  public void execute() {
    double throttle= driverController.getLeftY();
    double rotate= driverController.getRightX();
    
    if((throttle>0 && throttle<0.25) || (throttle<0 && throttle>-0.25)) {
      throttle=0; }
      else {throttle=0.35*throttle;}
    if ((rotate>0 && rotate<0.25) || (rotate<0 && rotate>-0.25)) {
      rotate=0;
    }
    rotate= 0.35*rotate;


    driveSubsystem.straightDrive(throttle, -rotate);
  }

  @Override
  public void end(boolean interrupted) {

  }


  @Override
  public boolean isFinished() {
    return false;
  }
}