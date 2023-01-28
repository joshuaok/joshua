
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
public class DriveSubsystem extends SubsystemBase {
  private static final WPI_TalonFX leftFrontMotor= RobotMap.leftFrontDrivePort;
  private static final WPI_TalonFX rightFrontMotor= RobotMap.rightFrontDrivePort;
  private static final WPI_TalonFX leftBackMotor= RobotMap.leftBackDrivePort;
  private static final WPI_TalonFX rightBackMotor= RobotMap.rightBackDrivePort;





  public double left_speed_cmd;
  public double right_speed_cmd;

  public double left_speed_feedback;
  public double right_speed_feedback;
  private double Meters_Per_Ticks;


  
  public void setModePercentVoltage(){
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);
  }
  public DriveSubsystem() {
    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID()); 

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftFrontMotor.configNominalOutputForward(0, 10);
    leftFrontMotor.configNominalOutputReverse(0, 10);
    leftFrontMotor.configPeakOutputForward(1, 10);
    leftFrontMotor.configPeakOutputReverse(-1, 10);
    leftFrontMotor.configNeutralDeadband(0.001, 10);

    rightFrontMotor.configNominalOutputForward(0, 10);
    rightFrontMotor.configNominalOutputReverse(0, 10);
    rightFrontMotor.configPeakOutputForward(1, 10);
    rightFrontMotor.configPeakOutputReverse(-1, 10);
    rightFrontMotor.configNeutralDeadband(0.001, 10);

    leftBackMotor.configNominalOutputForward(0, 10);
    leftBackMotor.configNominalOutputReverse(0, 10);
    leftBackMotor.configPeakOutputForward(1, 10);
    leftBackMotor.configPeakOutputReverse(-1, 10);
    leftBackMotor.configNeutralDeadband(0.001, 10);

    rightBackMotor.configNominalOutputForward(0, 10);
    rightBackMotor.configNominalOutputReverse(0, 10);
    rightBackMotor.configPeakOutputForward(1, 10);
    rightBackMotor.configPeakOutputReverse(-1, 10);
    rightBackMotor.configNeutralDeadband(0.001, 10);

    // Sets how much error is allowed
    leftFrontMotor.configAllowableClosedloopError(0, 0, 10);
    leftBackMotor.configAllowableClosedloopError(0, 0, 10);
    rightFrontMotor.configAllowableClosedloopError(0, 0, 10);
    rightBackMotor.configAllowableClosedloopError(0, 0, 10);

    leftFrontMotor.setNeutralMode(NeutralMode.Coast); 
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
    
    leftFrontMotor.setInverted(true);
    rightFrontMotor.setInverted(false);
    leftBackMotor.setInverted(true);
    rightBackMotor.setInverted(false);

    resetEncoders();
  }
  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }
  public void drive(double throttle, double rotate) {
    leftFrontMotor.set(throttle +rotate);
    rightFrontMotor.set(throttle - rotate);
    leftBackMotor.set(throttle + rotate);
    rightBackMotor.set(throttle - rotate);
  }
  public double getRightBackEncoderPosition(){
    return rightBackMotor.getSelectedSensorPosition();
  }
  public double getLeftBackEncoderPosition(){
    return leftBackMotor.getSelectedSensorPosition();
  }
  public double distanceTravelledinTick(){
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition())/2;
  }
  public double getLeftBackEncoderPositionVelocityMetersPerSecond()
  {
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorPosition()*10);
    leftVelocityMPS = leftVelocityMPS * Meters_Per_Ticks;
    return (leftVelocityMPS);
  }
  public double leftDistanceTravelledInMeters(){
    double left_dist=getRightBackEncoderPosition() * Meters_Per_Ticks;
    return left_dist;
  }
  public void stop() {
    drive(0,0);
  }


  public void straightDrive(double leftSpeed, double rightSpeed) {

   
    left_speed_feedback = getBackLeftEncoderVelocityMetersPerSecond();
    right_speed_feedback = getBackRightEncoderVelocityMetersPerSecond(); 

    left_speed_cmd = leftSpeed; // m/s
    right_speed_cmd = rightSpeed; // m/s

    double driveSpeedPer100MS = (DriveConstants.Ticks_Per_Meter * (1.0/1000.0) * 100.0); 

    leftBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*left_speed_cmd); 
    rightBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*right_speed_cmd);

  }

  public double getBackLeftEncoderVelocityMetersPerSecond() {
    double backLeftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity() * 10);
    backLeftVelocityMPS = backLeftVelocityMPS * DriveConstants.Meters_Per_Ticks;
    return (backLeftVelocityMPS);
  }

  public double getBackRightEncoderVelocityMetersPerSecond() {
    double backRightVelocityMPS = (rightBackMotor.getSelectedSensorVelocity() * 10); // /10
    backRightVelocityMPS = backRightVelocityMPS * DriveConstants.Meters_Per_Ticks;
    return (backRightVelocityMPS);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}