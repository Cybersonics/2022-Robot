package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

/**
 * An example command that uses an example subsystem.
 */
public class AutonIndexerCommand extends CommandBase {

  private Indexer _indexer;
  private Timer _timer;

  public AutonIndexerCommand(Indexer indexer) {
    _indexer = indexer;
    addRequirements(indexer);
  }

  public void start() {
    _indexer.forward();
  }

  public void stop() {
    _indexer.stop();
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
    start();
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
    return this._timer.hasElapsed(Constants.AutoRunTime);
  }
}