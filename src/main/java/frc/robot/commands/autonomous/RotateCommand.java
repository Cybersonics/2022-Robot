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
    private double _navStart;
    private double _navEnd;
    private Timer _timer;
    private NavXGyro _navXGyro;

    /**
     * Creates a new Drive.
     */
    public RotateCommand(Drive drive, double angle, NavXGyro navXGyro) {
      this._drive = drive;
      //this._navStart = this._drive.getNavAngle();
      this._degrees = angle;
      //this._navEnd = (this._navStart + this._degrees) % 360.0;
      this._navXGyro = navXGyro;

      
      this._drive.setDriveEncodersPosition(0);
      // System.out.println("Distance To Travel: " + this._distance);
		  addRequirements(this._drive);

    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      this._navXGyro.zeroNavHeading();
      this._navStart = this._navXGyro.getZeroAngle();
      //this._navStart = this._navXGyro.getNavAngle();
      this._navEnd = (this._navStart + this._degrees) % 360.0;
      this._timer = new Timer();
      _timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // double currentRotation = this._drive.getDriveEncoderAvg();
      // _distance = currentRotation + this._distance;
      this._drive.processInput(0.0, 0.0, 0.3/30.0, false);
      // System.out.println("Curent: " + currentRotation);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      _timer.stop();
      this._drive.processInput(0.0, 0.0, 0.0, false);

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      System.out.println("[RotateCommand#fin] Init Nav: " + this._navStart); 
      System.out.println("[RotateCommand#fin] End Nav(+-5): " + this._navEnd); 
      System.out.println("[RotateCommand#fin] GoTo Heading: " + (this._navStart - this._navXGyro.getNavAngle()));
      // if((this._navStart - this._navXGyro.getNavAngle()) >= (this._navEnd - 5) ||
      //  (this._navStart - this._navXGyro.getNavAngle()) >=  (this._navEnd + 5)) {
        if((this._navStart - this._navXGyro.getNavAngle()) >= (this._navEnd - 1)) {
        return true;
      }
      return false;
    }
  }