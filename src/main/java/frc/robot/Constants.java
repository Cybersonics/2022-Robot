// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
}
