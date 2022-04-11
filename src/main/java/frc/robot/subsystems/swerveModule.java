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

import frc.robot.utility.DriveLocation;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//import frc.robot.subsystems.setSwerveModule;

public class swerveModule extends SubsystemBase {
  /**
   * Creates a new swerveModule.
   */
  
  public double currentPosition;
  private TalonSRX steerMotor;
  private CANSparkMax driveMotor;
  private static final double RAMP_RATE = 0.5;//1.5;

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

  private boolean _driveCorrect;

  public swerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer) {

    //Create and configure a new Drive motor
    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveMotor.setInverted(invertDrive);
		driveMotor.setOpenLoopRampRate(RAMP_RATE);
		driveMotor.setIdleMode(IdleMode.kCoast); //changed to break at comp
    driveMotor.setSmartCurrentLimit(55);
 

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
    steerMotor.configAllowableClosedloopError(0, 2, 0);
    steerMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    steerMotor.setInverted(invertSteer);
    steerMotor.setSensorPhase(true);

    //Create the built in motor encoders
 
    driveMotorEncoder = driveMotor.getEncoder();
    driveMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
    driveMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    driveMotor.burnFlash();
    resetEncoders();
    //driveMotorEncoder.setPosition(0);

  }
   
  public void setSwerve(double angle, double speed, boolean driveCorrect) {

    this._driveCorrect = driveCorrect;
    
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
    double targetAngle = -angle; //-angle;
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
      if (!this._driveCorrect){
        if (Math.abs(deltaDegrees) > 90.0) {
          deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
          speed = -speed;
        }
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
  
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
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

  public double getSteerEncoder(){
    double curPosition = steerMotor.getSelectedSensorPosition(0);
    return curPosition;
  }
  
  public double getSteerEncDeg(){
    return (steerMotor.getSelectedSensorPosition() * 360.0 / this.encoderCountPerRotation) % 360.0;
  }
  
  public double getTurningPosition() {
    double steerEncoderRaw = getSteerEncoder();
    double turningEncoder = (steerEncoderRaw / this.encoderCountPerRotation) * 2 * Math.PI;
    return -turningEncoder;
  }

  public void resetEncoders() {
    driveMotorEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    double driveMotorSpeed = state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond;
    double steerMotorAngle = state.angle.getDegrees();
    setSwerve(steerMotorAngle, driveMotorSpeed, false);

  // driveMotor.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
  // turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  } 

  public void stop() {
    driveMotor.set(0);
    //steerMotor.set(0);
  }

  public void driveMotorRamp(boolean enableRamp){
    if (enableRamp) {
      driveMotor.setOpenLoopRampRate(RAMP_RATE);
    }
    else {
      driveMotor.setOpenLoopRampRate(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

