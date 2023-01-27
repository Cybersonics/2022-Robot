package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private final double MAX_SHOOTER_RATE = 0.75;//0.85
  public CANSparkMax _leftMotor;
  public CANSparkMax _rightMotor;
  private static Launcher instance;
  private RelativeEncoder _leftEncoder;
  private RelativeEncoder _rightEncoder;
  private SparkMaxPIDController _rightPIDController;
  private SparkMaxPIDController _leftPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private Launcher() {
    CommandScheduler.getInstance().registerSubsystem(this);
    setupRightMotor();
    setupLeftMotor();
    //_leftMotor.follow(_rightMotor);
     // PID coefficients
     kP = 0.00013;//0.0001; 
     kI = 0.0000002; //0.0000001;
     kD = 0.000000; //0.000001; 
     kIz = 0; 
     kFF = 0.00017;//0.00017;
     kMaxOutput = 1; 
     kMinOutput = -1;
     maxRPM = 5700;
     
       // set PID coefficients
    _rightPIDController.setP(kP);
    _rightPIDController.setI(kI);
    _rightPIDController.setD(kD);
    _rightPIDController.setIZone(kIz);
    _rightPIDController.setFF(kFF);
    _rightPIDController.setOutputRange(kMinOutput, kMaxOutput);

  }

  // Public Methods
  public static Launcher getInstance() {
    if (instance == null) {
      instance = new Launcher();
    }
    return instance;
  }

  public void calculatedLaunch(double speed) {
    _rightMotor.set(speed);
    _leftMotor.set(speed);
  }

  public void calculateReference(double reference) {
    _rightPIDController.setReference(reference, CANSparkMax.ControlType.kVelocity);
    _leftMotor.follow(_rightMotor, true);
  }

  public void start() {
    calculatedLaunch(MAX_SHOOTER_RATE);
  }

  public void stop() {
    calculatedLaunch(0);
  }

  public double getLeftEncoder(){
    return _leftEncoder.getVelocity();
  }

  public double getRightEncoder(){
    return _rightEncoder.getVelocity();
  }

  // Private Methods
  private void setupRightMotor() {
    _rightMotor = new CANSparkMax(Constants.R_Launcher, MotorType.kBrushless);
    _rightMotor.restoreFactoryDefaults();
    _rightMotor.setIdleMode(IdleMode.kCoast);
    _rightEncoder = _rightMotor.getEncoder();
    _rightPIDController = _rightMotor.getPIDController();
  }

  private void setupLeftMotor() {
    _leftMotor = new CANSparkMax(Constants.L_Launcher, MotorType.kBrushless);
    _leftMotor.restoreFactoryDefaults();
    _leftMotor.setIdleMode(IdleMode.kCoast);
    _leftMotor.setInverted(true);
    _leftEncoder = _leftMotor.getEncoder();
    //_leftPIDController = _leftMotor.getPIDController();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", getRightEncoder());
    //System.out.println("Left Shooter Encoder Speed: " + getLeftEncoder());
    //System.out.println("Right Shooter Encoder Speed: " + _shooter.getRightEncoder());
  }
}
