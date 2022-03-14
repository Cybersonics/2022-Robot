package frc.robot.commands.autonomous;

//import java.io.Console;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.math.controller.PIDController;

public class AutonDriveDistanceCommand extends CommandBase {

    private Drive _drive;
    private BallVision _ballVision;
    private double _distance;
    private Timer _timer;
    private boolean _enableVision;
    private double _forward;
    private double _strafe;
    private double _rotation;

    final double ANGULAR_P = 0.15;
    final double ANGULAR_I = 0.0;
    final double ANGULAR_D = 0.00;
  
    PIDController _turnPIDController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

    private double _targetYaw;
    private boolean _hasTarget;

    /**
     * Creates a new Drive.
     */
    public AutonDriveDistanceCommand(Drive drive, double distance, double forward, double strafe, double rotation, boolean enableVision, BallVision ballVision) {
      this._drive = drive;

      this._distance = distance * Constants.ROTATION_PER_INCH;
      this._enableVision = enableVision;
      this._ballVision = ballVision;
      this._forward = forward;
      this._strafe = strafe;
      this._rotation = rotation;
      
      this._drive.setDriveEncodersPosition(0);
      // System.out.println("Distance To Travel: " + this._distance);
		  addRequirements(this._drive);

    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // this._timer = new Timer();
      // _timer.start();
      this._drive.setDriveEncodersPosition(0);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
 
      // if (this._enableVision){
      //   this._hasTarget = this._ballVision.hasTargets();
      //   if (this._hasTarget){
      //     this._targetYaw = this._ballVision.getYawVal();
      //     this._rotation = _turnPIDController.calculate(this._targetYaw, 0);
      //   }
      //   this._drive.processInput(this._forward, this._strafe, this._rotation, false);
      // }
      // else {
        //System.out.println("Distance': " + this._drive.getDriveEncoderAvg());
        this._drive.processInput(this._forward, this._strafe, this._rotation, false);
      // }
 
      // System.out.println("Curent: " + currentRotation);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      // _timer.stop();
      this._drive.processInput(0 ,0, 0, false);
      this._drive.setDriveEncodersPosition(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      //System.out.println("'Distance': " + this._drive.getDriveEncoderAvg());
      if(this._distance <= Math.abs(this._drive.getDriveEncoderAvg())) {
        return true;
      }
      return false;
    }
  }