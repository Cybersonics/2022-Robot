// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretPositionCommand extends CommandBase {

  private Turret _turret;
  private double _position;
  /** Creates a new TurretPositionCommand. */
  public TurretPositionCommand(Turret turret, double position) {
    this._turret = turret;
    this._position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._turret.setTurretPosition(-0.8, this._position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    this._turret.stopTurretRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._position > this._turret.getTurretPosition();
  }
}
