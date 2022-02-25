// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretCommand extends CommandBase {

  private Turret _turret;
  private XboxController _controller;

  /** Creates a new TurretCommand. */
  public TurretCommand(Turret turret, XboxController controller) {
    this._turret = turret;
    this._controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._turret.rotateTurret(() -> this._controller.getRightX());
    // if (this._controller.getLeftTriggerAxis() > 0) {
    //   this._turret.rotateTurret(() -> this._controller.getLeftTriggerAxis());
    // }
    // if(this._controller.getRightTriggerAxis() > 0) {
    //   this._turret.rotateTurret(() -> this._controller.getRightTriggerAxis());
    // }
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
