package frc.robot.commands.autonomous;

import java.io.Console;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

    /**
     * Creates a new Drive.
     */
    public RotateCommand(Drive drive, double angle, NavXGyro navXGyro) {
      this._drive = drive;
      this._degrees = angle;
      this._navXGyro = navXGyro;

      
      this._drive.setDriveEncodersPosition(0);
      // System.out.println("Distance To Travel: " + this._distance);
		  addRequirements(this._drive);

    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      //this._navXGyro.zeroNavHeading();
      this._navZero = this._navXGyro.getZeroAngle();
      this._navStart = this._navXGyro.getNavAngle();
      //this._degOffset = this._navStart + this._degrees;
      this._degOffset = this._navZero + this._degrees;
      this._navEnd = (this._degOffset) % 360.0;
      if (this._navEnd < this._navStart){
        this._rotationDirection = true;
      }
      else {
        this._rotationDirection = false;
      }

      this._timer = new Timer();
      _timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      // if (this._degOffset > this._navStart){
      //if (this._degOffset < this._navZero){
      if (this._rotationDirection){
        this._drive.processInput(0.0, 0.0, 0.3/30.0, false, false);
        // System.out.println("Curent: " + currentRotation);
        //this._rotationDirection = true;
      }
      else {
        this._drive.processInput(0.0, 0.0, -0.3/30.0, false, false);
        //System.out.println("Curent: " + currentRotation);
        //this._rotationDirection = false;
      }

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      //_timer.stop();
      this._drive.processInput(0.0, 0.0, 0.0, false, false);

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      System.out.println("[RotateCommand#fin] Zero Nav: " + this._navZero); 
      //System.out.println("[RotateCommand#fin] Start Nav: " + this._navStart); 
      System.out.println("[RotateCommand#fin] End Nav(+-5): " + this._navEnd); 
      //System.out.println("[RotateCommand#fin] Angle Nav: " + this._navXGyro.getNavAngle()); 
      System.out.println("[RotateCommand#fin] GoTo Heading: " + (this._navZero + this._navXGyro.getNavAngle()));
      System.out.println("[RotateCommand#fin] rotationdirection: " + (this._rotationDirection));
   
      // if (this._rotationDirection){
      //   if((this._navStart - this._navXGyro.getNavAngle()) >= (this._navEnd - 1)) {
      //     return true;
      //   }
      // }
      // else {
      //   if((this._navStart - this._navXGyro.getNavAngle()) <= (this._navEnd - 1)) {
      //     return true;
      //   }
      // }
 
      if (this._rotationDirection){
        if((this._navZero + this._navXGyro.getNavAngle()) <= (this._navEnd - 1)) {
          return true;
        }
      }
      else {
        if((this._navZero + this._navXGyro.getNavAngle()) >= (this._navEnd - 1)) {
          return true;
        }
      }
      return false;
    }
  }