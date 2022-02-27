// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.DriveLocation;
import frc.robot.utility.SwerveDrive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Drive extends SubsystemBase {

  private static Drive instance;

  // private final double robotWidth;
  // private final double robotLength;

  // private SwerveDrive FL_Drive;
  // private SwerveDrive FR_Drive;
  // private SwerveDrive BL_Drive;
  // private SwerveDrive BR_Drive;

  // private static final double STEER_P = .5, STEER_I = 0.0, STEER_D = 0.0;
  // private static final int STATUS_FRAME_PERIOD = 20;

  // private static final double ENCODER_COUNT = 1024.0;

  // /**
  //  * Constructor to create class responsible for converting joystick interaction
  //  * to output on drives.
  //  * 
  //  * @param width  width of the robot in inches
  //  * @param length length of the robot in inches
  //  */
  // private Drive(double width, double length) {
  //   this.robotWidth = width;
  //   this.robotLength = length;

  //   this.FL_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.FrontLeft, 4.0)
  //       .DriveMotor(buildDriveMotor(Constants.FL_Drive_Id, true))
  //       .SteerMotor(buildSteerMotor(Constants.FL_Steer_Id, true))
  //       .Encoder(null, ENCODER_COUNT)
  //       .Build();

  //   this.FR_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.FrontRight, 4.0)
  //       .DriveMotor(buildDriveMotor(Constants.FR_Drive_Id, true))
  //       .SteerMotor(buildSteerMotor(Constants.FR_Steer_Id, false))
  //       .Encoder(null, ENCODER_COUNT)
  //       .Build();

  //   this.BL_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.BackLeft, 4.0)
  //       .DriveMotor(buildDriveMotor(Constants.BL_Drive_Id, true))
  //       .SteerMotor(buildSteerMotor(Constants.BL_Steer_Id, false))
  //       .Encoder(null, ENCODER_COUNT)
  //       .Build();

  //   this.BR_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.BackRight, 4.0)
  //       .DriveMotor(buildDriveMotor(Constants.BR_Drive_Id, true))
  //       .SteerMotor(buildSteerMotor(Constants.BR_Steer_Id, true))
  //       .Encoder(null, ENCODER_COUNT)
  //       .Build();
  // }

  // public void resetDriveEncoders() {
  //   this.FL_Drive.resetDriveEncoders();
  //   this.FR_Drive.resetDriveEncoders();
  //   this.BL_Drive.resetDriveEncoders();
  //   this.BR_Drive.resetDriveEncoders();
  // }

  // private CANSparkMax buildDriveMotor(int driveId, boolean invert) {
  //   var drive = new CANSparkMax(driveId, MotorType.kBrushless);
  //   drive.restoreFactoryDefaults();
  //   drive.setIdleMode(IdleMode.kBrake);
  //   drive.setOpenLoopRampRate(0.125);
  //   drive.setSmartCurrentLimit(60);
  //   drive.setInverted(invert);

  //   return drive;
  // }

  // private TalonSRX buildSteerMotor(int steerId, boolean invert) {
  //   var steer = new TalonSRX(steerId);
  //   steer.configFactoryDefault();
  //   steer.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
  //   steer.config_kP(0, STEER_P, 0);
  //   steer.config_kI(0, STEER_I, 0);
  //   steer.config_kD(0, STEER_D, 0);
  //   steer.config_IntegralZone(0, 100, 0);
  //   steer.configAllowableClosedloopError(0, 50, 0);
  //   steer.setNeutralMode(NeutralMode.Brake);
  //   steer.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
  //   steer.setInverted(invert);
  //   steer.setSensorPhase(false);

  //   return steer;
  // }

  // // Public Methods
  // public static Drive getInstance(double width, double length) {
  //   if (instance == null) {
  //     instance = new Drive(width, length);
  //   }
  //   return instance;
  // }


	private static swerveModule frontLeft;
	private static swerveModule backLeft;
	private static swerveModule frontRight;
	private static swerveModule backRight;

	//public static AHRS navX;
	public double heading;
	public double angle;

	private static final double WHEEL_BASE_LENGTH = 24;//23; // 30;
	private static final double WHEEL_BASE_WIDTH = 22;//21.5; // 28;

	//private static final double WHEEL_DIAMETER = 4.0;
	// TO DO: Correct equation that uses MAX_SPEED
	public static final double MAX_SPEED = 0.75; // Max speed is 0 to 1
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.7 * MAX_SPEED;

	public static final double DEADZONE = 0.06;

	public static final double OMEGA_SCALE = 1.0 / 30.0;

	private final boolean invertDrive = false;
	private final boolean invertSteer = true;

  private ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTab");

	private NetworkTableEntry lfSetAngle = driveTab.addPersistent("LF Set Angle", 0)
			.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
			.withPosition(0, 0).withSize(3, 1).getEntry();

	private NetworkTableEntry lbSetAngle = driveTab.addPersistent("LB Set Angle", 0)
			.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
			.withPosition(0, 1).withSize(3, 1).getEntry();

	private NetworkTableEntry rfSetAngle = driveTab.addPersistent("RF Set Angle", 0)
			.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
			.withPosition(4, 0).withSize(3, 1).getEntry();

	private NetworkTableEntry rbSetAngle = driveTab.addPersistent("RBack Set Angle", 0)
			.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
			.withPosition(4, 2).withSize(3, 1).getEntry();
	
	//swerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer)

	private Drive() {

		frontLeft = new swerveModule(Constants.FL_Steer_Id, Constants.FL_Drive_Id, invertDrive, invertSteer);

		backLeft = new swerveModule(Constants.BL_Steer_Id, Constants.BL_Drive_Id, invertDrive, invertSteer);

		frontRight = new swerveModule(Constants.FR_Steer_Id, Constants.FR_Drive_Id, invertDrive, invertSteer);

		backRight = new swerveModule(Constants.BR_Steer_Id, Constants.BR_Drive_Id, invertDrive, invertSteer);

	}

  // Public Methods
  // public static Drive getInstance(double width, double length) {
  //   if (instance == null) {
  //     instance = new Drive(width, length);
  //   }
  //   return instance;
  // }
	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}

		return instance;
	}

	public void stopFrontLeft() {
		frontLeft.stopDriveMotor();
	}

	public void stopBackLeft() {
		backLeft.stopDriveMotor();
	}

	public void stopFrontRight() {
		frontRight.stopDriveMotor();
	}

	public void stopBackRight() {
		backRight.stopDriveMotor();
	}

	public void setFrontLeft(double speed) {
		frontLeft.setDriveSpeed(speed);
	}

	public void setBackLeft(double speed) {
		backLeft.setDriveSpeed(speed);
	}

	public void setFrontRight(double speed) {
		frontRight.setDriveSpeed(speed);
	}

	public void setBackRight(double speed) {
		backRight.setDriveSpeed(speed);
	}

  public void processInput(double forward, double strafe, double omega, boolean deadStick) {
    double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
    SmartDashboard.putNumber("OmegaL2", omegaL2);
    double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
    SmartDashboard.putNumber("OmegaW2", omegaW2);

    SmartDashboard.putNumber("Forwrad", forward);
    SmartDashboard.putNumber("Strafe", strafe);

    // Compute the constants used later for calculating speeds and angles
    double A = strafe - omegaL2;
    double B = strafe + omegaL2;
    double C = forward - omegaW2;
    double D = forward + omegaW2;

    // Compute the drive motor speeds
    double speedFL = speed(B, D);
    double speedBL = speed(A, D);
    double speedFR = speed(B, C);
    double speedBR = speed(A, C);

    /*
		 * ... and angles for the steering motors Set the drive to face straight ahead
		 * and then either mechanically set the encoders to read zero, or mathematically
		 * correct the angle by reading the encoder value when the drive is pointed
		 * straight ahead and adding or subtracting that value from the reading
		 */

		/*
		 * Get offset values from the driver station using NetworkTables. Values are
		 * then input to "calibrate" the position of the drives mathematically rather
		 * then by mechanically positioning the drives and physically setting the
		 * encoder to zero.
		 */

		double lfOffset = lfSetAngle.getDouble(0.0);
		double lbOffset = lbSetAngle.getDouble(0.0);
		double rfOffset = rfSetAngle.getDouble(0.0);
		double rbOffset = rbSetAngle.getDouble(0.0);

		/*
		 * When drives are mechanically calibrated for zero position on encoders they
		 * can be at 90 degrees to the front of the robot. Adding or subtracting 90
		 * degrees to the steering calculation can be used offset for initial
		 * position/calibration of the drives.
		 * 
		 * For swerve and steer drives constants are 90 degrees out of phase when they
		 * are inserted in frames sideways. angleFL - 90 angleBL + 90 angleFR - 90
		 * angleBR + 90
		 */


    // Angles for the steering motors
    // When drives are calibrated for zero position on encoders
    // They are at 90 degrees to the front of the robot.
    // Subtract and add 90 degrees to steering calculation to offset for initial
    // position/calibration of drives.
		double angleFL = angle(B, D) + Constants.FL_STEER_OFFSET + lfOffset;
		double angleBL = angle(A, D) + Constants.BL_STEER_OFFSET + lbOffset;
		double angleFR = angle(B, C) + Constants.FR_STEER_OFFSET + rfOffset;
		double angleBR = angle(A, C) + Constants.BR_STEER_OFFSET + rbOffset;

    // double angleFL = angle(B, D);// - 90;
    // double angleBL = angle(A, D);// + 90;
    // double angleFR = angle(B, C) + 20;// - 90;
    // double angleBR = angle(A, C);// + 90;

    // Compute the maximum speed so that we can scale all the speeds to the range
    // [0.0, 1.0]
    double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    
    // SmartDashboard.putNumber("angleLF", angleFL);
    // SmartDashboard.putNumber("speedLF", speedFL);
    // SmartDashboard.putNumber("SpeedLF/MaxSpeed", speedFL / maxSpeed);
    if (deadStick) {

			// frontLeft.setSteerSpeed(0);
			frontLeft.setDriveSpeed(0);
			// backLeft.setSteerSpeed(0);
			backLeft.setDriveSpeed(0);
			// frontRight.setSteerSpeed(0);
			frontRight.setDriveSpeed(0);
			// backRight.setSteerSpeed(0);
			backRight.setDriveSpeed(0);

		} else {

			// Set each swerve module, scaling the drive speeds by the maximum speed
			frontLeft.setSwerve(angleFL, speedFL / maxSpeed);
			backLeft.setSwerve(angleBL, speedBL / maxSpeed);
			frontRight.setSwerve(angleFR, speedFR / maxSpeed);
			backRight.setSwerve(angleBR, speedBR / maxSpeed);
		}
    // this.FL_Drive.setAngleAndSpeed(angleLF, speedLF / maxSpeed);
    // this.BL_Drive.setAngleAndSpeed(angleLR, speedLR / maxSpeed);
    // this.FR_Drive.setAngleAndSpeed(angleRF, speedRF / maxSpeed);
    // this.BR_Drive.setAngleAndSpeed(angleRR, speedRR / maxSpeed);
  }

  private double speed(double val1, double val2) {
    return Math.hypot(val1, val2);
  }

  private double angle(double val1, double val2) {
    return Math.toDegrees(Math.atan2(val1, val2));
  }

  @Override()
  public void periodic() {
    // this.FL_Drive.outputToDashboard();
    // this.FR_Drive.outputToDashboard();
    // this.BR_Drive.outputToDashboard();
    // this.BL_Drive.outputToDashboard();
  }
}
