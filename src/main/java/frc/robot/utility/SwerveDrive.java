// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    private DriveLocation driveLocation;
    private double wheelDiameter;
    private CANSparkMax driveMotor;
    private TalonSRX steerMotor;
    private Encoder encoder;
    public double encoderCountPerRotation;

    public SwerveDrive(SwerveDriveBuilder builder) {
        this.driveLocation = builder.driveLocation;
        this.wheelDiameter = builder.wheelDiameter;
        this.driveMotor = builder.driveMotor;
        this.steerMotor = builder.steerMotor;
        this.encoder = builder.encoder;
        this.encoderCountPerRotation = builder.encoderCountPerRotation;
    }

    public DriveLocation getDriveLocation() {
        return this.driveLocation;
    }

    public CANSparkMax getDriveMotor() {
        return this.driveMotor;
    }

    public TalonSRX getSteerMotor() {
        return this.steerMotor;
    }

    public double getWheelDiameter() {
        return this.wheelDiameter;
    }

    public Encoder getEncoder() {
        return this.encoder;
    }

    public void resetDriveEncoders() {
        this.driveMotor.getEncoder().setPosition(0);
    }

    public void outputToDashboard() {
        SmartDashboard.putNumber(this.driveLocation.getName(), this.getSteerSensorPosition());
    }

    public void setAngleAndSpeed(double angle, double speed) {
        SmartDashboard.putNumber(this.driveLocation.getName()+" Angle", angle);
        SmartDashboard.putNumber(this.driveLocation.getName()+" Speed", speed);
        double currentPosition = getSteerSensorPosition();
        SmartDashboard.putNumber(this.driveLocation.getName()+" Current Position", currentPosition);
        double currentAngle = (currentPosition * 360.0 / this.encoderCountPerRotation) % 360.0;
        SmartDashboard.putNumber(this.driveLocation.getName()+" Current Angle", currentAngle);
        // The angle from the encoder is in the range [0, 360], but the swerve
        // computations
        // return angles in the range [-180, 180], so transform the encoder angle to
        // this range
        if (currentAngle > 180.0) {
            currentAngle -= 360.0;
        }
        // TODO: Properly invert the steering motors so this isn't necessary
        // This is because the steering encoders are inverted
        double targetAngle = -angle;
        double deltaDegrees = targetAngle - currentAngle;

        SmartDashboard.putNumber(this.driveLocation.getName()+" TargetAngle", targetAngle);
        SmartDashboard.putNumber(this.driveLocation.getName()+" DeltaDegrees", deltaDegrees);
        // If we need to turn more than 180 degrees, it's faster to turn in the opposite
        // direction
        // if (Math.abs(deltaDegrees) > 180.0) {
        //     deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
        //     speed = -speed;
        // }
        // // If we need to turn more than 90 degrees, we can reverse the wheel direction
        // // instead and
        // // only rotate by the complement

        // // if (Math.abs(speed) <= MAX_SPEED){
        // if (Math.abs(deltaDegrees) > 90.0) {
        //     deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
        //     speed = -speed;
        // }
        // }

        double targetPosition = currentPosition + deltaDegrees * this.encoderCountPerRotation / 360.0;
        
        SmartDashboard.putNumber(this.driveLocation.getName()+" Target Position", targetPosition);
        setSteerMotorPosition(targetPosition);
        setDriveMotorSpeed(speed);
    }

    private double getSteerSensorPosition() {
        // if(steerMotor instanceof CANSparkMax) {
        // return ((CANSparkMax)this.steerMotor).getEncoder().getPosition();
        // } else if (steerMotor instanceof TalonSRX) {
        return this.steerMotor.getSelectedSensorPosition(0);
        // } else {
        // throw new Error("MotorController Type not supported by code");
        // }
    }

    private void setSteerMotorPosition(double position) {
        // if(steerMotor instanceof CANSparkMax) {
        // ((CANSparkMax)this.steerMotor).getEncoder().setPosition(position);
        // } else if (steerMotor instanceof TalonSRX) {
        this.steerMotor.set(ControlMode.Position, position);
        // ((TalonSRX)this.steerMotor).set(ControlMode.Position, position);
        // } else {
        // throw new Error("MotorController Type not supported by code");
        // }
    }

    private void setDriveMotorSpeed(double speed) {
        // if(steerMotor instanceof CANSparkMax) {
        // ((CANSparkMax)this.driveMotor).set(speed);
        this.driveMotor.set(speed);
        // } else if (steerMotor instanceof TalonSRX) {
        // ((TalonSRX)this.driveMotor).set(ControlMode.PercentOutput, speed);
        // } else {
        // throw new Error("MotorController Type not supported by code");
        // }
    }

    public static class SwerveDriveBuilder {
        private final DriveLocation driveLocation;
        private final double wheelDiameter;

        private CANSparkMax driveMotor;
        private TalonSRX steerMotor;
        private Encoder encoder;
        private double encoderCountPerRotation;

        public SwerveDriveBuilder(DriveLocation location, double wheelDiameter) {
            this.driveLocation = location;
            this.wheelDiameter = wheelDiameter;
        }

        public SwerveDriveBuilder DriveMotor(CANSparkMax driveMotor) {
            this.driveMotor = driveMotor;
            return this;
        }

        public SwerveDriveBuilder Encoder(Encoder encoder, double encoderCountPerRotation) {
            this.encoder = encoder;
            this.encoderCountPerRotation = encoderCountPerRotation;
            return this;
        }

        public SwerveDriveBuilder SteerMotor(TalonSRX steerMotor) {
            this.steerMotor = steerMotor;
            return this;
        }

        public SwerveDrive Build() {
            SwerveDrive drive = new SwerveDrive(this);
            return drive;
        }
    }
}
