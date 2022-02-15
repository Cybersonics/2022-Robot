// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static String targetCamera = "mmal_service_16.1";
    public final static String ballCamera = "Microsoft_LifeCam_HD-3000";

    public final static int BluePipeline = 0;
    public final static int RedPipeline = 1;

<<<<<<< HEAD
    public final static int FL_Drive = 1;
    public final static int FR_Drive = 2;
    public final static int BL_Drive = 3;
    public final static int BR_Drive = 4;

    public final static int FL_Steer = 11;
    public final static int FR_Steer = 12;
    public final static int BL_Steer = 13;
    public final static int BR_Steer = 14;

    public final static int L_Launcher = 21;
    public final static int R_Launcher = 22;
=======
    public static final int FL_Drive_Id = 1;
    public static final int FL_Steer_Id = 11;
    public static final int FR_Drive_Id = 2;
    public static final int FR_Steer_Id = 12;
    public static final int BR_Drive_Id = 3;
    public static final int BR_Steer_Id = 13;
    public static final int BL_Drive_Id = 4;
    public static final int BL_Steer_Id = 14;

    public static final int XBOX_CONTROLLER = 2;

    public static final double ROBOT_WIDTH = 28.0;
    public static final double ROBOT_LENGTH = 30.0;
>>>>>>> c51b1c9fedb3abdd3a4a8314f243ae1f87e0432b
}
