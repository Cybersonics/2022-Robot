package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutonDriveDistanceCommand extends CommandBase {

  private Drive _drive;
  private double _distance;
  private double _forward;
  private double _strafe;
  private double _rotation;

  final double ANGULAR_P = 0.15;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.00;

  public AutonDriveDistanceCommand(Drive drive, double distance, double forward, double strafe, double rotation) {
    this._drive = drive;

    this._distance = distance * Constants.ROTATION_PER_INCH;
    this._forward = forward;
    this._strafe = strafe;
    this._rotation = rotation;

    this._drive.setDriveEncodersPosition(0);
    addRequirements(this._drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._drive.setDriveEncodersPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._drive.processInput(this._forward, this._strafe, this._rotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._drive.processInput(0, 0, 0, false);
    this._drive.setDriveEncodersPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._distance <= Math.abs(this._drive.getDriveEncoderAvg());
  }
}