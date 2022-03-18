// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.TargetVision;

public class TurretCommand extends CommandBase {

  private Turret _turret;
  private XboxController _controller;
  private TargetVision _targetVision;
  private double _targetYaw;
  private boolean _hasTarget;
  private double _targetDistance;

  public static final double TURRET_DEADZONE = 0.12;

  public TurretCommand(Turret turret, TargetVision targetVision, XboxController controller) {
    this._turret = turret;
    this._controller = controller;
    this._targetVision = targetVision;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(turret, targetVision);
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this._targetVision.hasTargets()) {
      this._hasTarget = true;
      this._targetYaw = this._targetVision.getYawVal();
      this._targetDistance = this._targetVision.getRange();
      double test = Units.metersToInches(this._targetDistance);
      // System.out.println("Distance': " + test);
    } else {
      this._hasTarget = false;
    }

    this._turret.rotateTurret(() -> this._controller.getRightX(), TURRET_DEADZONE, this._hasTarget, this._targetYaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
