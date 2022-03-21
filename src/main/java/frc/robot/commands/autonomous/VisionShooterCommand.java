// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.vision.TargetVision;
import frc.robot.utility.Interpolation;

public class VisionShooterCommand extends CommandBase {
  private Launcher _shooter;
  private TargetVision _targetVision;

  private Timer _timer;
  private double _autoTimeVal;

  /** Creates a new VisionShooterCommand. */
  public VisionShooterCommand(Launcher shooter, double autoTimeVal, TargetVision targetVision) {
    _shooter = shooter;
    _targetVision = targetVision;
    this._autoTimeVal = autoTimeVal;
    addRequirements(shooter, targetVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    _timer = new Timer();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetDistance = this._targetVision.getRange();
    double test = Units.metersToInches(targetDistance);
    double rpmReference = Interpolation.getReference(test);
    this._shooter.calculateReference(rpmReference);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stop();
    this._timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._timer.hasElapsed(this._autoTimeVal);
  }
}
