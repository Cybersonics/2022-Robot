package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.TargetVision;

public class ShooterCommand extends CommandBase {
  public double Pivot = 0.9;

  private Launcher _shooter;
  private Timer _timer;
  private boolean autoTime;
  private double _autoTimeVal;
  private Turret _turret;
  private TargetVision _targetVision;
  private boolean simple;
  private XboxController _controller;

  public ShooterCommand(Launcher shooter, double autoTimeVal) {
    _shooter = shooter;
    autoTime = true;
    this._autoTimeVal = autoTimeVal;
    simple = true;
    addRequirements(shooter);
  }

  public ShooterCommand(Launcher shooter, Turret turret, TargetVision targetVision, XboxController controller) {
    this._shooter = shooter;
    this._turret = turret;
    this._targetVision = targetVision;
    this._controller = controller;
    autoTime = false;
    simple = false;
    addRequirements(shooter);
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
    if (simple) {
    _shooter.start();
    } else {
      double targetDistance = this._targetVision.getRange();
      double test = Units.metersToInches(targetDistance);
      // System.out.println("Distance': " + test);
      if (this._controller.getRightBumper()) {
        if (this._turret.getTurretHoodPosition()) {
          this._shooter.calculatedLaunch(0.75);
        } else {
          this._shooter.calculatedLaunch(0.5);
        }
      } else {
        this._shooter.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stop();
    this._timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autoTime) {
      return this._timer.hasElapsed(this._autoTimeVal);
    } else {
      return false;
    }
  }
}