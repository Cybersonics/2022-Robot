// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.TargetVision;
import frc.robot.utility.Interpolation;

public class VisionShooterCommand extends CommandBase {
  private Launcher _shooter;
  private Turret _turret;
  private TargetVision _targetVision;

  private Timer _timer;
  private double _autoTimeVal;
  
  public static final double TURRET_DEADZONE = 0.12;

  /** Creates a new VisionShooterCommand. */
  public VisionShooterCommand(Launcher shooter, double autoTimeVal, TargetVision targetVision, Turret turret) {
    _shooter = shooter;
    _targetVision = targetVision;
    _turret = turret;
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
    if (this._targetVision.hasTargets()) {
      double targetYaw = this._targetVision.getYawVal();
      double targetDistance = this._targetVision.getRange();
      double test = Units.metersToInches(targetDistance);
      double angleReference = Interpolation.getAngleReference(test);
      double rpmReference = Interpolation.getRPMReference(test);
      this._turret.rotateTurret(() -> 0, TURRET_DEADZONE, true, targetYaw);
      this._turret.setTurretPosition(angleReference);
      this._shooter.calculateReference(rpmReference);
    }
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
