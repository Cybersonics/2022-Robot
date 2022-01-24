// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;

public class BallVision extends SubsystemBase {
    private PhotonCamera camera;
    
    public BallVision() {
        this.camera = new PhotonCamera(Constants.ballCamera);
        
        var alliance = DriverStation.getAlliance();
        if(alliance == Alliance.Blue) {
            this.camera.setPipelineIndex(Constants.BluePipeline);
        } else if(alliance == Alliance.Red){
            this.camera.setPipelineIndex(Constants.RedPipeline);
        }
    }

    public List<PhotonTrackedTarget> getTargets() {
        List<PhotonTrackedTarget> targets = null;
        var results = this.camera.getLatestResult();
        if(results.hasTargets()) {
            targets = results.getTargets();
        }
        return targets;
    }
}
