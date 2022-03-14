// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoVisionCommand extends CommandBase {
  private Timer _timer;
  private TargetVision _targetVision;
  private boolean autoRoutine;

  /** Creates a new AutoVisionCommand. */
  public AutoVisionCommand(TargetVision targetVision) {

    this._targetVision = targetVision;
    this.autoRoutine = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_targetVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _targetVision.cameraLEDToggleOn();
    this.autoRoutine = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.autoRoutine;
    //return false;
  }
}
