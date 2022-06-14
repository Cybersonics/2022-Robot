// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class IndexerCommand extends CommandBase {

  private Indexer _indexer;
  private XboxController _xboxController;
  private boolean autoRoutine;
  private double _speed;
  private Timer _timer;
  private double _runTime;

  /** Creates a new IndexerCommand. */
  public IndexerCommand(Indexer indexer, XboxController driveController) {
    this._indexer = indexer;
    this._xboxController = driveController;
    this.autoRoutine = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_indexer);
  }

  public IndexerCommand(Indexer indexer, double speed) {
    this._indexer = indexer;
    this._speed = speed;
    this.autoRoutine = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_indexer);
  }

  public IndexerCommand(Indexer indexer, double speed, double runTime) {
    this._indexer = indexer;
    this._speed = speed;
    this.autoRoutine = true;
    this._runTime = runTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer = new Timer();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  /*
      The indexing command first looks to see if the robot is in autonomous mode.
      During autonomous the speed of the indexer is controlled by sending a speed
      directly to the indexer.
      If the sensor at the front of the indexer is triggered and there is no input 
      from the operator the indexer automatically cycles the ball in until the ball
      clears the sensor. When clear and there is no input from the operator the 
      indexer will take a value of zero from the operator controller.
  */
  @Override
  public void execute() {
    if (autoRoutine) {
      this._indexer.manualControl(this._speed);
    } else {
      if (!this._indexer.getBallTrip() && Math.abs(_xboxController.getLeftY()) < 0.07) {
        this._indexer.manualControl(-.65);
      } else {
        this._indexer.manualControl(() -> _xboxController.getLeftY());
      }
    }
  }

  public void stop() {
    this._indexer.manualControl(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
    this._timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autoRoutine) {
      // return this._timer.hasElapsed(Constants.AutoRunTime);
      return this._timer.hasElapsed(_runTime);
    } else {
      return false;
    }
  }
}