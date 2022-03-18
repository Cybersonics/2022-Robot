// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  private Timer _timer;
  private Intake _intake;
  private boolean autoRoutine;
  private double _speed;
  private XboxController _xboxController;
  private double _time_val;

  public IntakeCommand(Intake intake, XboxController xboxController) {
    this._intake = intake;
    this._xboxController = xboxController;
    this.autoRoutine = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_intake);
  }

  /** Creates a new Intake. */
  public IntakeCommand(Intake intake, double speed, double time_val) {
    this._intake = intake;
    this._speed = speed;
    this._time_val = time_val;
    this.autoRoutine = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_intake);
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
    if (autoRoutine) {
      this._intake.manualControl(this._speed);
    } else {
      _intake.manualControl(() -> _xboxController.getRightY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._intake.manualControl(0);
    this._timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autoRoutine) {
      return this._timer.hasElapsed(this._time_val);
    } else {
      return false;
    }
  }
}
