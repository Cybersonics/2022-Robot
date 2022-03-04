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
  private XboxController _controller;
  private TargetVision _targetVision;

  /** Creates a new TurretCommand. */
  public TurretCommand(Turret turret, XboxController controller) {
    this._turret = turret;
    this._controller = controller;
    //this._targetVision = targetVision;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(turret, targetVision);
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this._turret.rotateTurret(() -> this._controller.getRightX());
    if (this._controller.getRightStickButton()) {
      // if (this._controller.getLeftTriggerAxis() > 0) {
        this._targetVision.cameraLEDOn();
      //   this._turret.rotateTurret(() -> this._controller.getLeftTriggerAxis());
        double yaw = this._targetVision.getYawVal();
      // }
        this._turret.setPosition(yaw);
      // if(this._controller.getRightTriggerAxis() > 0) {
      } else {
      //   this._turret.rotateTurret(() -> this._controller.getRightTriggerAxis());
        this._turret.rotateTurret(() -> this._controller.getRightX());
      // }
      }
    
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
