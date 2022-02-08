// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveCommand extends CommandBase {

  
  private Drive _drive;
  private XboxController driveController;


  /** Creates a new Drive. */
  public DriveCommand(Drive drive, XboxController driveController) {
    this._drive = drive;
    this.driveController = driveController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._drive.processInput(
      () -> driveController.getLeftY() * .2,
      () -> driveController.getLeftX() * .2,
      () -> driveController.getRightX() * .2
      );
      
      // this._drive.processInput(
      //   () -> 0.0,
      //   () -> 0.0,
      //   () -> 0.0
      //   );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
