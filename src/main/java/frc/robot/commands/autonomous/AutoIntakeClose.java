// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntakeClose extends CommandBase {
  /** Creates a new AutoIntakeDeploy. */
  private Pneumatics _pneumatics;
  private Timer _timer;

  public AutoIntakeClose(Pneumatics pnematics) {
    // Use addRequirements() here to declare subsystem dependencies.
    this._pneumatics = pnematics;
    addRequirements(pnematics);
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
    this._pneumatics.intakeClose();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._timer.hasElapsed(0.5);
  }
}
