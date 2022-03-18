// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TargetVision extends SubsystemBase {
    private PhotonCamera camera;
    public static TargetVision instance;
    private double yawVal = 0;
    private double pitchVal = 0;
    private double skewVal = 0;
    private double areaVal = 0;
    private boolean hasTarget = false;
    private boolean isLEDEnabled = false;


    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28);
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(105);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

    public TargetVision() {
        this.camera = new PhotonCamera(Constants.targetCamera);
        this.camera.setPipelineIndex(Constants.Tape01);
    }

    public static TargetVision getInstance() {
        if (instance == null) {
            instance = new TargetVision();
        }
        return instance;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget = true;

            SmartDashboard.putNumber("Yaw Value", yawVal);
            SmartDashboard.putNumber("Pitch Value", pitchVal);
            SmartDashboard.putNumber("Area Value", areaVal);
            SmartDashboard.putBoolean("LEDs OnOff", this.isLEDEnabled);
        } else {
            this.hasTarget = false;
        }
        if (isLEDEnabled) {
            cameraLEDOn();
            // Set driver mode to off.
            camera.setDriverMode(false);
        } else {
            cameraLEDOff();
            // Set driver mode to on.
            camera.setDriverMode(true);
        }
    }

    public double getYawVal() {
        return this.yawVal;
    }

    public double getPitchVal() {
        return this.pitchVal;
    }

    public double getSkewVal() {
        return this.skewVal;
    }

    public double getAreaVal() {
        return this.areaVal;
    }

    public boolean hasTargets() {
        return this.hasTarget;
    }

    public void cameraLEDOff() {
        this.camera.setLED(VisionLEDMode.kOff);
    }

    // Note that this will just turn the LEDs on
    public void cameraLEDOn() {
        this.camera.setLED(VisionLEDMode.kOn);
    }

    public void cameraLEDBlink() {
        this.camera.setLED(VisionLEDMode.kBlink);
    }

    public void cameraLEDToggle() {
        if (isLEDEnabled) {
            isLEDEnabled = false;
        } else {
            isLEDEnabled = true;
        }
    }
    
    public boolean isLEDEnabled() {
        return isLEDEnabled;
    }

    public void setLEDEnabled(boolean isLEDEnabled) {
        this.isLEDEnabled = isLEDEnabled;
    }

    public void cameraLED() {
        // Note that this will use the value of the pipeline for the intensity
        this.camera.setLED(VisionLEDMode.kDefault);
    }

    public double getRange() {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(getPitchVal()));
        return range;
    }
}
