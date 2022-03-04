// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetVision extends SubsystemBase {
    private PhotonCamera camera;
    public static TargetVision instance;
    private double yawVal=0;
    private double pitchVal=0;
    private double skewVal=0;
    private double areaVal=0;
    private boolean hasTarget = false;
    
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();

            // SmartDashboard.putNumber("Yaw Value", yawVal);
            // SmartDashboard.putNumber("Pitch Value", pitchVal);
            // SmartDashboard.putNumber("Area Value", areaVal);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


    public double getYawVal(){
        return this.yawVal;
    }

    public double getPitchVal(){
        return this.pitchVal;
    }

    public double getSkewVal(){
        return this.skewVal;
    }

    public double getAreaVal(){
        return this.areaVal;
    }

    public boolean hasTargets(){
        return this.hasTarget;
    }

    public void cameraLEDOff() {
        this.camera.setLED(VisionLEDMode.kOff);
    }

    public void cameraLEDOn() {
        //Note that this will just turn the LEDs on
        this.camera.setLED(VisionLEDMode.kOn);
    }

    public void cameraLEDBlink() {
        this.camera.setLED(VisionLEDMode.kBlink);
    }

    public void cameraLED() {
        //Note that this will use the value of the pipeline for the intensity
        this.camera.setLED(VisionLEDMode.kDefault);
       
    }
}
