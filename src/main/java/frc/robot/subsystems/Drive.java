// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Drive extends SubsystemBase {

  private static Drive instance;

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

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

	public static final double OMEGA_SCALE = 1.0 / 30.0;

	private final boolean invertDrive = true;//false;
	private final boolean invertSteer = true;
	private NavXGyro _gyro;

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

	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics,
		new Rotation2d(0));
	  
	
	//swerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer)

	private Drive(NavXGyro gyro) {

		this._gyro = gyro;

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
  public Pose2d getPose() {
	return odometer.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
	odometer.resetPosition(pose, this._gyro.getRotation2d());
}

public static Drive getInstance(NavXGyro gyro) {
		if (instance == null) {
			instance = new Drive(gyro);
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
    //SmartDashboard.putNumber("OmegaL2", omegaL2);
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
    // double speedFL = speed(B, D);
    // double speedBL = speed(A, D);
    // double speedFR = speed(B, C);
    // double speedBR = speed(A, C);
	double speedFL = speed(B, C);
    double speedBL = speed(A, C);
    double speedFR = speed(B, D);
    double speedBR = speed(A, D);

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
		double angleFL = angle(B, C)+ lfOffset;// + Constants.FL_STEER_OFFSET;//+ lfOffset;
		double angleBL = angle(A, C)+ lbOffset;// + Constants.BL_STEER_OFFSET;// + lbOffset;
		double angleFR = angle(B, D)+ rfOffset;// + Constants.FR_STEER_OFFSET;// + rfOffset;
		double angleBR = angle(A, D)+ rbOffset;// + Constants.BR_STEER_OFFSET;// + rbOffset;
		// double angleFL = angle(B, D) + Constants.FL_STEER_OFFSET + lfOffset;
		// double angleBL = angle(A, D) + Constants.BL_STEER_OFFSET + lbOffset;
		// double angleFR = angle(B, C) + Constants.FR_STEER_OFFSET + rfOffset;
		// double angleBR = angle(A, C) + Constants.BR_STEER_OFFSET + rbOffset;

    // Compute the maximum speed so that we can scale all the speeds to the range
    // [0.0, 1.0]
    double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    
	SmartDashboard.putNumber("angleLF", angleFL);
    SmartDashboard.putNumber("speedLF", speedFL);
    SmartDashboard.putNumber("CurAngle LF", frontLeft.getSteerEncDeg());
    // SmartDashboard.putNumber("angleRF", angleFR);
    // SmartDashboard.putNumber("speedRF", speedFR);
    // SmartDashboard.putNumber("angleLR", angleBL);
    // SmartDashboard.putNumber("speedLR", speedBL);
    // SmartDashboard.putNumber("angleRR", angleBR);
    // SmartDashboard.putNumber("speedRR", speedBR);
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
	//getSteerEncoderVal();
  }

  private double speed(double val1, double val2) {
    return Math.hypot(val1, val2);
  }

  private double angle(double val1, double val2) {
    return Math.toDegrees(Math.atan2(val1, val2));
  }

  public double[] getDriveEncoders() {
		double[] values = new double[] {
			frontLeft.getDriveEncoder(),
			backLeft.getDriveEncoder(),
			frontRight.getDriveEncoder(),
			backRight.getDriveEncoder()
		};

		return values;
	}
	
	public double getDriveEncoderAvg() {
		// double driveFL = frontLeft.getDriveEncoder();
		// double driveBL = backLeft.getDriveEncoder();
		// double driveFR = frontRight.getDriveEncoder();
		// double driveBR = backRight.getDriveEncoder();
		double driveFL = Math.abs(frontLeft.getDriveEncoder());
		double driveBL = Math.abs(backLeft.getDriveEncoder());
		double driveFR = Math.abs(frontRight.getDriveEncoder());
		double driveBR = Math.abs(backRight.getDriveEncoder());
		return (driveFL + driveFR + driveBL + driveBR) / 4.0;
	}

	public void setDriveEncodersPosition(double position) {
		frontLeft.setDriveEncoder(position);
		backLeft.setDriveEncoder(position);
		frontRight.setDriveEncoder(position);
		backRight.setDriveEncoder(position);
	}

	public void getSteerEncoderVal(){
		SmartDashboard.putNumber("angleLF", frontLeft.getSteerEncoder());
		SmartDashboard.putNumber("angleLB", backLeft.getSteerEncoder());
		SmartDashboard.putNumber("angleRF", frontRight.getSteerEncoder());
		SmartDashboard.putNumber("angleRB", backRight.getSteerEncoder());
	}

	// public static double[] getEncoderVal() {
	// 	double[] values = new double[] { frontLeft.getAnalogIn(), backLeft.getAnalogIn(), frontRight.getAnalogIn(),
	// 			backRight.getAnalogIn() };

	// 	return values;
	//}

  @Override()
  public void periodic() {

	odometer.update(this._gyro.getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
	backRight.getState());
	SmartDashboard.putNumber("Robot Heading", this._gyro.getHeading());
	SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
		//******** */
	// Uncomment following line to physically reset encoders position to zero state.
	// getSteerEncoderVal();

	//SmartDashboard.putNumber("Angle Back Left", backLeft.getSteerEncoder());
    // this.FL_Drive.outputToDashboard();
    // this.FR_Drive.outputToDashboard();
    // this.BR_Drive.outputToDashboard();
    // this.BL_Drive.outputToDashboard();
  }

  public void stopModules() {
	frontLeft.stop();
	frontRight.stop();
	backLeft.stop();
	backRight.stop();
}

public void disableRamping(){
	frontLeft.driveMotorRamp(false);
	frontRight.driveMotorRamp(false);
	backLeft.driveMotorRamp(false);
	backRight.driveMotorRamp(false);
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
	SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond/2);
	frontLeft.setDesiredState(desiredStates[0]);
	frontRight.setDesiredState(desiredStates[1]);
	backLeft.setDesiredState(desiredStates[2]);
	backRight.setDesiredState(desiredStates[3]);
}
}
