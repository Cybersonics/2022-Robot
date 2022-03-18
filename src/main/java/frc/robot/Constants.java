// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kSteerMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
        public static final double kPSteer = 0.5;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(25);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(27);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftSteerMotorPort = 7;
        public static final int kBackLeftSteerMotorPort = 1;
        public static final int kFrontRightSteerMotorPort = 5;
        public static final int kBackRightSteerMotorPort = 3;

        public static final boolean kFrontLeftSteerEncoderReversed = true;
        public static final boolean kBackLeftSteerEncoderReversed = true;
        public static final boolean kFrontRightSteerEncoderReversed = true;
        public static final boolean kBackRightSteerEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
}
