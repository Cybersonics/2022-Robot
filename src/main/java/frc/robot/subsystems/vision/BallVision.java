// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallVision extends SubsystemBase {
    private PhotonCamera camera;
    public static BallVision instance;
    private double yawVal = 0;
    private double pitchVal = 0;
    private double skewVal = 0;
    private double areaVal = 0;
    private boolean hasTarget = false;

    public BallVision() {
        this.camera = new PhotonCamera(Constants.ballCamera);

        var alliance = DriverStation.getAlliance();
        if (alliance == Alliance.Blue) {
            this.camera.setPipelineIndex(Constants.BluePipeline);
        } else if (alliance == Alliance.Red) {
            this.camera.setPipelineIndex(Constants.RedPipeline);
        }
    }

    public static BallVision getInstance() {
        if (instance == null) {
            instance = new BallVision();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget = true;

            SmartDashboard.putNumber("Ball Yaw Value", yawVal);

        } else {
            this.hasTarget = false;
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
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

    public List<PhotonTrackedTarget> getTargets() {
        List<PhotonTrackedTarget> targets = null;
        var results = this.camera.getLatestResult();
        if (results.hasTargets()) {
            targets = results.getTargets();
        }
        return targets;
    }
}
