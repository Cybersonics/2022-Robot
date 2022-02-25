// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

public enum DriveLocation {
    FrontLeft ("Front Left"),
    FrontRight ("Front Right"),
    BackLeft ("Back Left"),
    BackRight ("Back Right");

    private final String name;
    private DriveLocation(String name) {
        this.name = name;
    }
    public String getName() {
        return this.name;
    }
}
