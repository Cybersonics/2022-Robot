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
}
