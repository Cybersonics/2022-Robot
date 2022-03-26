package frc.robot.commands.autonomous;

//import java.io.Console;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
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
    private double originHeading = 0.0;
    private boolean _usingFieldCentric;
    private NavXGyro _navXGyro;

    final double ANGULAR_P = 0.15;
    final double ANGULAR_I = 0.0;
    final double ANGULAR_D = 0.00;
  
    //PIDController _turnPIDController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

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
      this._rotation = rotation / Constants.ROBOT_LENGTH;
      this._usingFieldCentric = false;
      
      this._drive.setDriveEncodersPosition(0);
      // System.out.println("Distance To Travel: " + this._distance);
		  addRequirements(this._drive);

    }

    public AutonDriveDistanceCommand(Drive drive, double distance, double forward, double strafe, double rotation, boolean enableVision, BallVision ballVision, NavXGyro navXGyro) {
      this._drive = drive;

      this._distance = distance * Constants.ROTATION_PER_INCH;
      this._enableVision = enableVision;
      this._ballVision = ballVision;
      this._forward = forward;
      this._strafe = strafe;
      this._rotation = rotation / Constants.ROBOT_LENGTH;

      this._usingFieldCentric = true;
      this._navXGyro = navXGyro;
      
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
      if(this._usingFieldCentric) originHeading = _navXGyro.getZeroAngle();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
 
      if(this._usingFieldCentric) {
        final double originCorrection = Math.toRadians(originHeading - _navXGyro.getNavAngle());
        final double temp = this._forward * Math.cos(originCorrection) - this._strafe * Math.sin(originCorrection);
        this._strafe = this._strafe * Math.cos(originCorrection) + this._forward * Math.sin(originCorrection);
        this._forward = temp;
        System.out.println("Strafe: " + this._strafe + "\tForward: " + this._forward + "\tOrigin Heading: " + originHeading
         + "\t NavXGryo Angle: " + _navXGyro.getNavAngle());
      }
      
      this._drive.processInput(this._forward, this._strafe, this._rotation, false);
 
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