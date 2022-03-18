package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;

public class RotateCommand extends CommandBase {

  private Drive _drive;
  private double _degrees;
  private double _navZero;
  private double _degOffset;
  private double _navStart;
  private double _navEnd;
  private Timer _timer;
  private NavXGyro _navXGyro;
  private boolean _rotationDirection;

  public RotateCommand(Drive drive, double angle, NavXGyro navXGyro) {
    this._drive = drive;
    this._degrees = angle;
    this._navXGyro = navXGyro;

    this._drive.setDriveEncodersPosition(0);
    addRequirements(this._drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._navZero = this._navXGyro.getZeroAngle();
    this._navStart = this._navXGyro.getNavAngle();
    this._degOffset = this._navZero + this._degrees;
    this._navEnd = (this._degOffset) % 360.0;
    if (this._navEnd < this._navStart) {
      this._rotationDirection = true;
    } else {
      this._rotationDirection = false;
    }

    this._timer = new Timer();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this._rotationDirection) {
      this._drive.processInput(0.0, 0.0, 0.3 / 30.0, false);
    } else {
      this._drive.processInput(0.0, 0.0, -0.3 / 30.0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._drive.processInput(0.0, 0.0, 0.0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this._rotationDirection) {
      if ((this._navZero + this._navXGyro.getNavAngle()) <= (this._navEnd - 1)) {
        return true;
      }
    } else {
      if ((this._navZero + this._navXGyro.getNavAngle()) >= (this._navEnd - 1)) {
        return true;
      }
    }
    return false;
  }
}