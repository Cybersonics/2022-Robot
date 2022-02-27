/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.math.controller.PIDController; //Use for Roborio PID
//import edu.wpi.first.math.MathUtil; // Use for RoboRio PID
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.subsystems.setSwerveModule;

public class swerveModule extends SubsystemBase {
  /**
   * Creates a new swerveModule.
   */
  
  public double currentPosition;
  private TalonSRX steerMotor;
  private CANSparkMax driveMotor;
  private static final double RAMP_RATE = 1.5;

  //Use the following two line if using PID in RoboRIO
  //private static final double STEER_P = .0035, STEER_I = 0.00003, STEER_D = 0.0000;
  //private PIDController steerPID;

  private AnalogInput analogIn; //Set up analog input for Roborio
 
  private RelativeEncoder driveMotorEncoder; //Set up integrated Drive motor encoder in Spark Max/Neo
  
  private double lastEncoderVal = 0;
  private double numTurns = 0;
  private double maxEncoderVolts = 3.3;
  //private static final double STEER_MOTOR_RATIO = 18; //Ratio between steering motor and Swerve pivot

  private double loopCounter = 0;
  private static final double MAXSTEERERROR = 5;
  private static final double STEER_P = 3.0, STEER_I = 0.0, STEER_D = 0.1;
  private static final int STATUS_FRAME_PERIOD = 5;

  public double encoderCountPerRotation = 1024;

  public swerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer) {

    //Create and configure a new Drive motor
    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveMotor.setInverted(invertDrive);
		driveMotor.setOpenLoopRampRate(RAMP_RATE);
		driveMotor.setIdleMode(IdleMode.kCoast);

    //Create and configure an analog input on a roborio port
    //analogIn = new AnalogInput(analogNum);

    //Create and configure a new Steering motor
    steerMotor = new TalonSRX(steerNum);
    steerMotor.configFactoryDefault();
    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steerMotor.config_kP(0, STEER_P, 0);
    steerMotor.config_kI(0, STEER_I, 0);
    steerMotor.config_kD(0, STEER_D, 0);
    steerMotor.config_IntegralZone(0, 100, 0);
    steerMotor.configAllowableClosedloopError(0, 5, 0);
    steerMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    steerMotor.setInverted(invertSteer);
    steerMotor.setSensorPhase(true);

    //Create the built in motor encoders
 
    driveMotorEncoder = driveMotor.getEncoder();

    driveMotorEncoder.setPosition(0);

  }
   
  public void setSwerve(double angle, double speed) {
    
    //double currentAngle = getAnalogIn() % 360.0; // Use for RoboRio PID
    //double currentSteerPosition = getSteerMotorEncoder();
    //double currentAngle = currentSteerPosition % 360.0;
    //double currentAngle = getSteerMotorEncoder();
    //double targetAngle = angle; //-angle;
    //double deltaDegrees = targetAngle - currentAngle;

    //SmartDashboard.putNumber(this.driveLocation.getName()+" Angle", angle);
    //SmartDashboard.putNumber(this.driveLocation.getName()+" Speed", speed);
    double currentPosition = steerMotor.getSelectedSensorPosition(0);
    //SmartDashboard.putNumber(this.driveLocation.getName()+" Current Position", currentPosition);
    double currentAngle = (currentPosition * 360.0 / this.encoderCountPerRotation) % 360.0;
    //SmartDashboard.putNumber(this.driveLocation.getName()+" Current Angle", currentAngle);
    double targetAngle = angle; //-angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite
    // direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }

    // If we need to turn more than 90 degrees, we can reverse the wheel direction
    // instead and
    // only rotate by the complement
    //if (Math.abs(speed) <= MAX_SPEED){
      if (Math.abs(deltaDegrees) > 90.0) {
      	deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      	speed = -speed;
      }
	  //}
    //Add change in position to current position
    //double targetPosition = currentAngle + deltaDegrees; 
    double targetPosition = currentPosition + ((deltaDegrees/360) * encoderCountPerRotation);
    //Scale the new position to match the motor encoder
    //double scaledPosition = (targetPosition / (360/STEER_MOTOR_RATIO)); 

    //steerPID.setSetpoint(targetPosition); // Use for RoboRio PID
    //double steerOutput = steerPID.calculate(currentAngle); // Use for RoboRio PID
    //steerOutput = MathUtil.clamp(steerOutput, -1, 1); // Use for RoboRio PID


    driveMotor.set(speed);
    steerMotor.set(ControlMode.Position, targetPosition);
   


    //Use Dashboard items to help debug
    // SmartDashboard.putNumber("Incoming Angle", angle);
    // SmartDashboard.putNumber("CurAngle", currentAngle);
    // SmartDashboard.putNumber("TargetAngle", targetAngle);
    // SmartDashboard.putNumber("currentSteerPosition", currentSteerPosition);
    // SmartDashboard.putNumber("DeltaDegrees", deltaDegrees);
    // SmartDashboard.putNumber("TargetPosition", targetPosition);
    // SmartDashboard.putNumber("Steer Output", scaledPosition);
    // SmartDashboard.putNumber("currentPosition", currentAngle);
    // SmartDashboard.putNumber("Steer Output", steerOutput);
  }

  
  //Get the built in Spark/Neo Drive motor encoder position. Value is in motor revolutions.
  public double getDriveEncoder() {
    return driveMotorEncoder.getPosition();
  }
  
  //Set the position value of the Spark/Neo Drive motor encoder position. Position is in 
  //motor revolutions.
  public void setDriveEncoder(double position) {
    driveMotorEncoder.setPosition(position);
  }
  
  //Set the drive motor speed from -1 to 1 
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }
  
  //Get the drive motor speed.
  public double getDriveSpeed() {
    return driveMotor.get();
  }
  
  public void stopDriveMotor() {
    driveMotor.stopMotor();
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

