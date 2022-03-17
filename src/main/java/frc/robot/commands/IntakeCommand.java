// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeCommand extends CommandBase {

  private Timer _timer;
  private Intake _intake;
  private boolean autoRoutine;
  private double _speed;
  private XboxController _xboxController;

  /** Creates a new Intake. */
  public IntakeCommand(Intake intake, XboxController xboxController) {
    this._intake = intake;
    this._xboxController = xboxController;
    this.autoRoutine = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_intake);
  }

    /** Creates a new Intake. */
    public IntakeCommand(Intake intake, double speed) {
      this._intake = intake;
      this._speed = speed;
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
    //System.out.println("'speed': " + this._speed);
    if (autoRoutine){
      this._intake.manualControl(this._speed);
    }
    else{
      _intake.manualControl(() -> _xboxController.getRightY());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autoRoutine){
      return this._timer.hasElapsed(2.5);
    }
    else {
      return false;
    }
  }
}
