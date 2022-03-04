package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.vision.BallVision;
import frc.robot.subsystems.vision.TargetVision;

public class AutonomousRoutines {
    private Drive _drive;
    private Launcher _launcher;
    private TargetVision _targetVision;
    private BallVision _ballVision;
    private Intake _intake;
    private Indexer _indexer;
    private NavXGyro _navxGyro;
    private Turret _turret;

    public AutonomousRoutines(Drive drive, Indexer indexer, Launcher launcher, Turret turret, NavXGyro navXGyro) {
        this._drive = drive;
        this._indexer = indexer;
        this._launcher = launcher;
        this._turret = turret;
        //this._targetVision = targetVision;
        this._navxGyro = navXGyro;
    }

    public Command DoNothing() {
        return new DoNothingCommand();
    }

    public Command testMove() {
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 30, 0.0, 0.4, 0.0, true)
        );
    }

    public Command testRotate() {
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, 30, this._navxGyro)
        );
    }

    public Command testShooter() {
        return new SequentialCommandGroup(
            new ShooterCommand(this._launcher)
        );
    }

    public Command testIndexer() {
        return new SequentialCommandGroup(
            new IndexerCommand(this._indexer, -0.5)
        );
    }

    public Command getCenterRotateFireAndMove(){
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 30, 0.4, 0.0, 0.0, false),
            new RotateCommand(this._drive, 90, this._navxGyro),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0),
                new ShooterCommand(this._launcher)
            ),
            new AutonDriveDistanceCommand(this._drive, 10,0.0, 0.4, 0.0, true)
        );
    }

    public Command getLeftRotateFireAndMove(){
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, 30, this._navxGyro),
            new AutonDriveDistanceCommand(this._drive, 25, 0.0, 0.4, 0.0, false),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0),
                new ShooterCommand(this._launcher)
            ),
            new AutonDriveDistanceCommand(this._drive, 10 ,0.0, 0.4, 0.0, true)
        );
    }

    public Command getRightRotateFireAndMove(){
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, 70, this._navxGyro),
            new AutonDriveDistanceCommand(this._drive, 30, 0.0, -0.4, 0.0, false),
            new ParallelCommandGroup(
                new IndexerCommand(this._indexer, -1.0),
                new ShooterCommand(this._launcher)
            ),
            new AutonDriveDistanceCommand(this._drive, 15 ,0.0, -0.4, 0.0, true)
        );
    }
}