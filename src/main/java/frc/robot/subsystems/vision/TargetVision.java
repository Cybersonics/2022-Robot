// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import frc.robot.Constants;

public class TargetVision extends SubsystemBase {
    private static TargetVision instance;
    private PhotonCamera camera;

    private TargetVision() {
        this.camera = new PhotonCamera(Constants.targetCamera);
    }

    public static TargetVision getInstance() {
        if(instance == null) {
            instance = new TargetVision();
        }
        return instance;
    }

    public double getOffset() {
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        } else {
            return 0;
        }
    }

    public void lightsOn() {
        this.camera.setLED(VisionLEDMode.kOn);
    }

    public void lightsOff() {
        this.camera.setLED(VisionLEDMode.kOff);
    }
}
