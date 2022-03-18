package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.*;
import frc.robot.utility.Interpolation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class ShooterCommand extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //public double ShooterSpeed = 0.55;//0.8;//.85
  public double Pivot = 0.9;

  private Launcher _shooter;
  private Timer _timer;
  private boolean autoTime;
  private double _autoTimeVal;
  private Turret _turret;
  private TargetVision _targetVision;
  private boolean simple;
  private XboxController _controller;

  private double rpmReference = 3400;

  /**
   * constructor method
   *
   * @param subsystem The subsystem used by this command.
  //  */
  // public ShooterCommand(Launcher shooter) {
  //   _shooter = shooter;
  //   autoTime = false; 
  //   simple = true;
  //   //CommandScheduler.getInstance().requiring(shooter);
  //   addRequirements(shooter);

  // }

  public ShooterCommand(Launcher shooter, double autoTimeVal) {
    _shooter = shooter;

    //CommandScheduler.getInstance().requiring(shooter);
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
    //CommandScheduler.getInstance().requiring(shooter);
    autoTime = false; 
    simple = false;
    addRequirements(shooter);
  }

  public void fire() {
      _shooter.start();
  }

  public void stop() {
      _shooter.stop();
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
    rpmReference = SmartDashboard.getNumber("RPM Speed", rpmReference);
    SmartDashboard.putNumber("RPM Speed", rpmReference);
    if (simple){
      fire();
    }
    else {
      if (this._controller.getRightBumper()){
        if (this._turret.getTurretHoodPosition()){
          this._shooter.calculatedLaunch(0.75);
        } else {
          this._shooter.calculatedLaunch(0.5);
        }
      } else if (this._controller.getLeftBumper()) {
        double targetDistance = this._targetVision.getRange();
        double test = Units.metersToInches(targetDistance);
        rpmReference = Interpolation.getReference(test);
        this._shooter.calculateReference(rpmReference);
      } else {
        this._shooter.stop();
      }
    }
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
    if (autoTime){
      return this._timer.hasElapsed(this._autoTimeVal);
    }
    else {
      return false;
    }
  }
}