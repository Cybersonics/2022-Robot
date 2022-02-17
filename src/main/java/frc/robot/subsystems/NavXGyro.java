// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class NavXGyro extends SubsystemBase { 

  private static NavXGyro instance;

  public static AHRS navX;
  public static double zeroHeading;
  public static double zeroAngle;

  /** Creates a new NavXGyro. */
  private NavXGyro() {
    navX = new AHRS(SPI.Port.kMXP);

    zeroHeading = getNavHeading();
    zeroAngle = getNavAngle();
    System.out.println("Setup ZeroAngle " + zeroAngle);
    
  }
  
  // Public Methods
  public static NavXGyro getInstance() {
    if (instance == null) {
      instance = new NavXGyro();
    }
    return instance;
  }

  public double getNavHeading() {
		double heading = navX.getFusedHeading();
		return heading;
	}

	public double getNavAngle() {
		double angle = navX.getAngle();
		return angle;
	}

	public void zeroNavHeading() {
		//navX.zeroYaw();
    navX.reset();
    zeroHeading = getNavHeading();
    zeroAngle = getNavAngle();
    System.out.println("ZeroHeading: " + zeroHeading);
    System.out.println("ZeroAngle: " + zeroAngle);  
  }

  public double getZeroHeading(){
    return zeroHeading;
  }

  public double getZeroAngle(){
    return zeroAngle;
  }
}