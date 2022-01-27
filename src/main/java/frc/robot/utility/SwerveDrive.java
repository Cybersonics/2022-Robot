// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveDrive {

    private DriveLocation driveLocation;
    private MotorController driveMotor;
    private BaseTalon steerMotor;
    private Encoder encoder;

    public SwerveDrive(SwerveDriveBuilder builder) {
        this.driveLocation = builder.driveLocation;
        this.driveMotor = builder.driveMotor;
        this.steerMotor = builder.steerMotor;
        this.encoder = builder.encoder;
    }

    public DriveLocation getDriveLocation() {
        return this.driveLocation;
    }

    public MotorController getDriveMotor() {
        return this.driveMotor;
    }

    public BaseTalon getSteerMotor() {
        return this.steerMotor;
    }

    public Encoder getEncoder() {
        return this.encoder;
    }

    public static class SwerveDriveBuilder {
        private final DriveLocation driveLocation;
        private MotorController driveMotor;
        private BaseTalon steerMotor;
        private Encoder encoder;

        public SwerveDriveBuilder(DriveLocation location) {
            this.driveLocation = location;
        }

        public SwerveDriveBuilder DriveMotor(MotorController driveMotor) {
            this.driveMotor = driveMotor;
            return this;
        }

        public SwerveDriveBuilder Encoder(Encoder encoder) {
            this.encoder = encoder;
            return this;
        }

        public SwerveDriveBuilder SteerMotor(MotorController steerMotor) {
            throw new UnsupportedOperationException();
        }

        public SwerveDriveBuilder SteerMotor(BaseTalon steerMotor) {
            this.steerMotor = steerMotor;
            return this;
        }

        public SwerveDrive Build() {
            SwerveDrive drive = new SwerveDrive(this);
            return drive;
        }
    }
}
