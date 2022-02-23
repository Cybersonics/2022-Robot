// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  private Intake _intake;
  private XboxController _xboxController;
  /** Creates a new Intake. */
  public IntakeCommand(Intake intake, XboxController xboxController) {
    this._intake = intake;
    this._xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.manualControl(() -> _xboxController.getRightY());
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
