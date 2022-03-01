// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.TargetVision;

public class TurretCommand extends CommandBase {

  private Turret _turret;
  private TargetVision _targetVision;
  private XboxController _controller;

  /** Creates a new TurretCommand. */
  public TurretCommand(Turret turret, TargetVision targetVision, XboxController controller) {
    this._turret = turret;
    this._targetVision = targetVision;
    this._controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, targetVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this._controller.getRightStickButton()) {
      this._targetVision.lightsOn();
      double yaw = this._targetVision.getOffset();
      this._turret.setPosition(yaw);
    } else {
      this._turret.rotateTurret(() -> this._controller.getRightX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._targetVision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
