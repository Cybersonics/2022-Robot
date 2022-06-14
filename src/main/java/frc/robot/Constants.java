// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static String targetCamera = "mmal_service_16.1";
    public final static String ballCamera = "Microsoft_LifeCam_HD-3000";

    public final static int BluePipeline = 0;
    public final static int RedPipeline = 1;
    public final static int Tape01 = 0;

    public final static int L_Launcher = 21;
    public final static int R_Launcher = 22;
    public final static int Turret = 30;

    public static final int FL_Drive_Id = 1;
    public static final int FL_Steer_Id = 11;
    public static final int FR_Drive_Id = 2;
    public static final int FR_Steer_Id = 12;
    public static final int BR_Drive_Id = 3;
    public static final int BR_Steer_Id = 13;
    public static final int BL_Drive_Id = 4;
    public static final int BL_Steer_Id = 14;

    public static final int LEFT_STICK = 0;
    public static final int RIGHT_STICK = 1;
    public static final int DRIVE_CONTROLLER = 1;
    public static final int OP_CONTROLLER = 2;

    public static final double ROBOT_WIDTH = 28.0;
    public static final double ROBOT_LENGTH = 30.0;

    public static final int INDEXER_ID = 23;
    public static final int INTAKER_ID = 24;
    public static final int R_CLIMBER_ID = 41;
    public static final int L_CLIMBER_ID = 42;

    public static final int L_INTAKE_OUT = 0;
    public static final int L_INTAKE_IN = 1;
    public static final int R_INTAKE_OUT = 6;
    public static final int R_INTAKE_IN = 7;

    public static final int L_CLIMBER_OUT = 2;
    public static final int L_CLIMBER_IN = 3;
    public static final int R_CLIMBER_OUT = 4;
    public static final int R_CLIMBER_IN = 5;

    public final static int FL_STEER_OFFSET=0;
    public final static int BL_STEER_OFFSET=0;
    public final static int FR_STEER_OFFSET=0;
    public final static int BR_STEER_OFFSET=0;

    public final static double ROTATION_PER_INCH = .551;

    public final static double AutoRunTime = 3.0;

    public final static double TURRET_CAMERA_HEIGHT = 0.67945;  //this is in meters
    public static final double TARGET_HEIGHT = 2.6416;  //this is in meters

    public final static double[] MIN_RPM_REFERENCE = {100, 2900};
    public final static double[] MAX_RPM_REFERENCE = {190, 3800};
    public final static double[][] distanceReference = {
        MIN_RPM_REFERENCE, // Distance: 100 RPM: 2900
        {115, 3000},   // Distance: 115 RPM: 3000
        {130, 3100},   // Distance: 130 RPM: 3050
        {145, 3250},   // Distance: 130 RPM: 3225
        {160, 3400},   // Distance: 160 RPM: 3400
        {175, 3650}, // Distance: 175 RPM: 3600
        MAX_RPM_REFERENCE  // Distance: 190 RPM: 3800
    };

    public final static double[] MIN_ANGLE_REFERENCE = {100, 0};
    public final static double[] MAX_ANGLE_REFERENCE = {190, 50}; 
    public final static double[][] angleReference = {
        MIN_ANGLE_REFERENCE, // Distance: 100 Angle: 0
        {115, 7}, // Distance: 115 Angle: 7
        {130, 15},   // Distance: 130 Angle: 15
        {145, 15}, // Distance: 145 Angle: 15
        {160, 35},   // Distance: 160 Angle: 35
        {175, 40}, // Distance: 175 Angle: 40
        MAX_ANGLE_REFERENCE  // Distance: 190 Angle: 50
    };
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.923;
        public static final double kTurningMotorGearRatio = 1 / 1024;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }
    public static final class FrameConstants {
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(22);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(24);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6;//Orig 5
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =  2 * Math.PI;//2*Math.PI;//orig 2*2*Math.PI

        // public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        // public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        //         kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        // public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        // public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = FrameConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = FrameConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5;//10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;//orig 3
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;// 2 //Orig 4
        public static final double kPXController = 5;//8;//Orig 1.5
        public static final double kPYController = 5;//8;//Orig 1.5
        public static final double kPThetaController = 5;//5;//0.5;//Orig 3

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        // public static final double kGoToPointLinearP = 0;
        // public static final double kGoToPointLinearF = 0.5;
        // public static final double kGoToPointAngularP = 0;
        // public static final double kGoToPointAngularF = 0;

        // public static final double maxTrajectoryOverrunSeconds = 3;
        // public static final double kMaxDistanceMetersError = 0.1;
        // public static final double kMaxAngleDegreesError = 5;


    }

}
