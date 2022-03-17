package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.BallVision;
import frc.robot.subsystems.vision.TargetVision;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

public class AutonomousRoutines {
    private Drive _drive;
    private Launcher _launcher;
    private TargetVision _targetVision;
    private BallVision _ballVision;
    private Intake _intake;
    private Indexer _indexer;
    private NavXGyro _navxGyro;
    private Turret _turret;
    private Pneumatics _pneumatics;
    

    public AutonomousRoutines(Drive drive, 
                            Indexer indexer, 
                            Launcher launcher, 
                            Turret turret, 
                            NavXGyro navXGyro, 
                            Pneumatics pneumatics, 
                            TargetVision targetVision,
                            BallVision ballVision,
                            Intake intake) {
        
        this._drive = drive;
        this._indexer = indexer;
        this._launcher = launcher;
        this._turret = turret;
        this._targetVision = targetVision;
        this._ballVision = ballVision;
        this._navxGyro = navXGyro;
        this._pneumatics = pneumatics;
        this._intake = intake;

    }
    public AutonomousRoutines(Drive drive, 
                            Indexer indexer, 
                            Launcher launcher, 
                            Turret turret, 
                            NavXGyro navXGyro, 
                            Pneumatics pneumatics, 
                            TargetVision targetVision,
                            Intake intake) {

this._drive = drive;
this._indexer = indexer;
this._launcher = launcher;
this._turret = turret;
this._targetVision = targetVision;
this._navxGyro = navXGyro;
this._pneumatics = pneumatics;
this._intake = intake;

}

    public Command DoNothing() {
        return new DoNothingCommand();
    }

    public Command testMove() {
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 30, 0.0, 0.4, 0.0, false, this._ballVision)
        );
    }

    public Command testRotate() {
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, -90, this._navxGyro),
            new RotateCommand(this._drive, -96, this._navxGyro)
        );
    }

    public Command testShooter() {
        return new SequentialCommandGroup(
            new ShooterCommand(this._launcher, Constants.AutoRunTime)
        );
    }

    public Command testIndexer() {
        return new SequentialCommandGroup(
            new IndexerCommand(this._indexer, -0.5, 5)
        );
    }

    public Command testAutoIntakeDeploy() {
        return new SequentialCommandGroup(
            new AutoIntakeDeploy(this._pneumatics),
            new ParallelCommandGroup(
                new AutoVisionCommand(this._targetVision),
                new IntakeCommand(this._intake, 1.0),
                new IndexerCommand(this._indexer, -1.0,Constants.AutoRunTime),
                new ShooterCommand(this._launcher, Constants.AutoRunTime)
            )
        );
    }

    public Command getCenterRotateFireAndMove(){
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 30, 0.4, 0.0, 0.0, false, this._ballVision),
            //new RotateCommand(this._drive, 90, this._navxGyro),
            new RotateCommand(this._drive, -75, this._navxGyro),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0, Constants.AutoRunTime),
                new ShooterCommand(this._launcher, Constants.AutoRunTime)
            ),
            new AutonDriveDistanceCommand(this._drive, 10,0.0, 0.4, 0.0, false, this._ballVision)
        );
    }

    public Command getLeftRotateFireAndMove(){
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, -30, this._navxGyro),
            new AutonDriveDistanceCommand(this._drive, 25, 0.0, 0.4, 0.0, false, this._ballVision),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0, Constants.AutoRunTime),
                new ShooterCommand(this._launcher, Constants.AutoRunTime)
            ),
            new AutonDriveDistanceCommand(this._drive, 10 ,0.0, 0.4, 0.0, false, this._ballVision)
        );
    }

    public Command getRightRotateFireAndMove(){
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, -70, this._navxGyro),
            new AutonDriveDistanceCommand(this._drive, 30, 0.0, -0.4, 0.0, false, this._ballVision),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0, Constants.AutoRunTime),
                new ShooterCommand(this._launcher, Constants.AutoRunTime)
            ),
            new AutonDriveDistanceCommand(this._drive, 15 ,0.0, -0.4, 0.0, false, this._ballVision)
        );
    }

    public Command getCenterTwoBall(){
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 33, 0.4, 0.0, 0.0, false, this._ballVision),
            new RotateCommand(this._drive, -75, this._navxGyro),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0, Constants.AutoRunTime),
                new ShooterCommand(this._launcher, Constants.AutoRunTime)
            ),
            new AutoIntakeDeploy(this._pneumatics),
            //new RotateCommand(this._drive, -100, this._navxGyro),
            new ParallelCommandGroup(
                new AutoVisionCommand(this._targetVision),
                //new AutonDriveDistanceCommand(this._drive, 40,-0.4, -0.38, 0.0, true, this._ballVision),
                new AutonDriveDistanceCommand(this._drive, 40,-0.4, -0.2, 0.0, true, this._ballVision),
                new IntakeCommand(this._intake, 1.0),
                new IndexerCommand(this._indexer, -1.0, 7),
                new ShooterCommand(this._launcher, 7)
            )
        );
    }

    public Command getRightThreeBall(){
        //THIS IS NOT COMPLETE WHATSOEVER
        return new SequentialCommandGroup(
            
            //rotate and deploy intake
            new ParallelCommandGroup(
                new AutoVisionCommand(this._targetVision),
                new RotateCommand(this._drive, 60, this._navxGyro),
                new AutoIntakeDeploy(this._pneumatics)
            ),

            //drive to ball and intake ball
            new ParallelCommandGroup(
                new AutonDriveDistanceCommand(this._drive, 17, -0.4, 0.0, 0.0, false, this._ballVision),
                new IntakeCommand(this._intake, 1.0)
            ),

            //shoot the balls
            new ParallelCommandGroup(
                new ShooterCommand(this._launcher, 5),
                new IndexerCommand(this._indexer, -1.0, 5),
                new IntakeCommand(this._intake, 1.0)
            ),

            //rotate to face the third ball
            new AutonDriveDistanceCommand(this._drive, 3, 0.4, 0.0, 0.0, false, this._ballVision),
            new RotateCommand(this._drive, 185, this._navxGyro)
        );
    }
}