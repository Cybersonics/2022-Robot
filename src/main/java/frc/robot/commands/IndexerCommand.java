// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends CommandBase {

  private Indexer _indexer;
  private XboxController _xboxController;

  /** Creates a new IndexerCommand. */
  public IndexerCommand(Indexer indexer, XboxController driveController) {
    this._indexer = indexer;
    this._xboxController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._indexer.manualControl(() -> _xboxController.getLeftY());
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
