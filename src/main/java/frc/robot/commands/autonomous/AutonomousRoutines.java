package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TurretCommand;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FrameConstants;
import java.util.List;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.*;
import java.util.HashMap;

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
  private TrajectorySetup _trajectorySetup;
  public HashMap<String, Command> eventMap;

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

    /*
      Create the trajectory setup and Control PIDs required
    */
      this._trajectorySetup = new TrajectorySetup();

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

    /*
      Create the trajectory setup and Control PIDs required
    */
    this._trajectorySetup = new TrajectorySetup();
    eventMap = new HashMap<>();
    // eventMap.put("shooterStart", new ShooterSetRPM(5000));
    // eventMap.put("intakeDown", new IntakeDown());
    // eventMap.put("intakeOn", new IntakeRun());
    // eventMap.put("intakeOff", new IntakeStop());
    // eventMap.put("turnToTarget", new DriveTurnToTarget());
    // eventMap.put("shoot", new ShooterShoot());

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
        new IntakeCommand(this._intake, 1.0, 3.0),
        new IndexerCommand(this._indexer, -1.0, 3.0),
        new ShooterCommand(this._launcher, 3.0)
      )
    );
  }

  public Command testTurretRotate() {
    return new TurretPositionCommand(this._turret, -160);
  }

  public Command testRotateMove() {
    return new SequentialCommandGroup(
      new AutonDriveDistanceCommand(this._drive, 50, 0.4, 0.0, 0.0, false, this._ballVision, this._navxGyro)
      // new RotateCommand(this._drive, -60, this._navxGyro),
      // new AutonDriveDistanceCommand(this._drive, 50, -0.4, 0.0, 0.0, false,
      // this._ballVision, this._navxGyro)
    );
  }
    
  public Command getCenterRotateFireAndMove() {
    return new SequentialCommandGroup(
      new AutoVisionCommand(this._targetVision),
      new AutonDriveDistanceCommand(this._drive, 1, -0.4, 0.0, 0.0, false, this._ballVision),
      // new RotateCommand(this._drive, 90, this._navxGyro),
      new RotateCommand(this._drive, -82, this._navxGyro),
      new ParallelCommandGroup(
        new IndexerCommand(this._indexer, -1.0, 3.0),
        new VisionShooterCommand(this._launcher, 3.0, this._targetVision, this._turret)
      ),
      new AutonDriveDistanceCommand(this._drive, .1, 0.0, 0.4, 0.0, false, this._ballVision)
    );
  }

  public Command getLeftRotateFireAndMove() {
    return new SequentialCommandGroup(
      new RotateCommand(this._drive, -30, this._navxGyro),
      new AutonDriveDistanceCommand(this._drive, 25, 0.0, 0.4, 0.0, false, this._ballVision),
      new ParallelCommandGroup(
        new IndexerCommand(this._indexer, -1.0, 3.0),
        new ShooterCommand(this._launcher, 3.0)
      ),
      new AutonDriveDistanceCommand(this._drive, 10, 0.0, 0.4, 0.0, false, this._ballVision)
    );
  }


  public Command getRightRotateFireAndMove() {
    return new SequentialCommandGroup(
      new RotateCommand(this._drive, -70, this._navxGyro),
      new AutonDriveDistanceCommand(this._drive, 1, 0.0, -0.4, 0.0, false, this._ballVision),
      new ParallelCommandGroup(
        new IndexerCommand(this._indexer, -1.0, 3.0),
        new ShooterCommand(this._launcher, 3.0)
      ),
      new AutonDriveDistanceCommand(this._drive, .3, 0.0, -0.4, 0.0, false, this._ballVision)
    );
  }

  public Command getCenterTwoBall() {
    return new SequentialCommandGroup(
      new AutoVisionCommand(this._targetVision),
      new AutonDriveDistanceCommand(this._drive, 1, -0.4, 0.0, 0.0, false, this._ballVision),
      new RotateCommand(this._drive, -82, this._navxGyro), // drive, -90, navyGryo
      new ParallelCommandGroup(
        new IndexerCommand(this._indexer, -1.0, 3.0),
        new VisionShooterCommand(this._launcher, 3, this._targetVision, this._turret)
      ),
      new AutoIntakeDeploy(this._pneumatics),
      new ParallelCommandGroup(
        new AutonDriveDistanceCommand(this._drive, 1, 0.4, -0.3, 0.0, true, this._ballVision),
        new IntakeCommand(this._intake, 1.0, 7),
        new IndexerCommand(this._indexer, -1.0, 7),
        new VisionShooterCommand(this._launcher, 7, this._targetVision, this._turret)
      )
    );
  }

  public Command getLeftTwoBall() {
    return new SequentialCommandGroup(
      new AutoVisionCommand(this._targetVision),
      new AutonDriveDistanceCommand(this._drive, 40, 0.4, 0.0, 0.0, false, this._ballVision),
      new RotateCommand(this._drive, -82, this._navxGyro), // drive, -90, navyGryo
      new ParallelCommandGroup(
        new IndexerCommand(this._indexer, -1.0, 3.0),
        new VisionShooterCommand(this._launcher, 3, this._targetVision, this._turret)
      ),
      new AutoIntakeDeploy(this._pneumatics),
      // new RotateCommand(this._drive, -100, this._navxGyro),
      new ParallelCommandGroup(
        // new AutoVisionCommand(this._targetVision),
        new AutonDriveDistanceCommand(this._drive, 40, -0.4, -0.2, 0.0, true, this._ballVision),
        // new AutonDriveDistanceCommand(this._drive, 40, -0.4, 0.1, 0.0, true,
        // this._ballVision),
        new IntakeCommand(this._intake, 1.0, 7),
        new IndexerCommand(this._indexer, -1.0, 7),
        new VisionShooterCommand(this._launcher, 7, this._targetVision, this._turret)
      )
    );
  }
 
  public Command getRightFourBall() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoVisionCommand(this._targetVision),
        new TurretPositionCommand(this._turret, -87),
        new RotateCommand(this._drive, 140, this._navxGyro),
        new AutoIntakeDeploy(this._pneumatics)
      ),

      new ParallelCommandGroup(
        new AutonDriveDistanceCommand(this._drive, 40, -0.4, 0.1, 0.0, false, this._ballVision),
        new IntakeCommand(this._intake, 1.0, 3.5),
        new IndexerCommand(this._indexer, -0.3, 4)
      ),

      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new AutoIntakeClose(this._pneumatics),
          new IndexerCommand(this._indexer, 0.7, 0.5)
        )
      ),

      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 3, this._targetVision, this._turret),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          new IndexerCommand(this._indexer, -1, 3.0)
        )   
      ),
            
      new AutoIntakeDeploy(this._pneumatics),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(           
          new AutonDriveDistanceCommand(this._drive, 20, -0.5, -0.1, 0.0, false, this._ballVision),
          new WaitCommand(.5),
          new AutonDriveDistanceCommand(this._drive, 40, -0.5, -0.2, 0.0, false, this._ballVision)
        ),
        new IntakeCommand(this._intake, 1.0, 3.0),
        new IndexerCommand(this._indexer, -0.3, 3.25),
        new TurretPositionCommand(this._turret, -110)
      ),
      //new AutonDriveDistanceCommand(this._drive, 24, 0.4, 0.0, 0.0, false, this._ballVision),

      new ParallelCommandGroup(                        
        new VisionShooterCommand(this._launcher, 5, this._targetVision, this._turret),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new IndexerCommand(this._indexer, -1, 4.0)
        )
      )
    );
  }

  public Command testAutoMove(){
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.FrameConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(-.3,-.05),
        new Translation2d(-.5,-.1)),
        new Pose2d(-1.2, 0.0, Rotation2d.fromDegrees(180)
      ),
      trajectoryConfig
    );

    // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
    //         //_drive.getPose(),
            
    //         new Pose2d(-1.2, 0.0, new Rotation2d(Math.PI)),
    //         List.of(
    //         new Translation2d(-0.2,0.0),
    //         new Translation2d(-0.5,-0.05)),
    //         new Pose2d(-2.2, 0.05, Rotation2d.fromDegrees(180)),
    //         trajectoryConfig);
    

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
      trajectory1,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      _drive::setModuleStates,
      _drive
    );

    // SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
    //         trajectory2,
    //         //swerveSubsystem::getPose,
    //         _drive::getPose,
    //         Constants.kDriveKinematics,
    //         xController,
    //         yController,
    //         thetaController,
    //         _drive::setModuleStates,
    //         _drive);

    // 5. Add some init and wrap-up, and return everything

    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoVisionCommand(this._targetVision),
        new TurretPositionCommand(this._turret, -45),//-87
        new AutoIntakeDeploy(this._pneumatics),
        new InstantCommand(() -> _drive.resetOdometry(trajectory1.getInitialPose()))
      ),

      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          swerveControllerCommand1,
          //swerveControllerCommand2, 
          new InstantCommand(() -> _drive.stopModules()),          
          new IndexerCommand(this._indexer, -0.3, 4)
        ),
        new VisionShooterCommand(this._launcher, 7, this._targetVision, this._turret)
      )
    );

    // return new SequentialCommandGroup(
    
    //  new InstantCommand(() -> _drive.resetOdometry(trajectory.getInitialPose())),
    //  swerveControllerCommand,
    //  new InstantCommand(() -> _drive.stopModules()));

  }

  /*
        Use this as a base model for path planning
  */
  public Command RightPathPlanner2Ball(){
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.FrameConstants.kDriveKinematics);
    
    // 2. Generate trajectory
    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", 4, 3);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      testPath,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      _trajectorySetup.xController,
      _trajectorySetup.yController,
      _trajectorySetup.thetaController,
      _drive::setModuleStates,
      _drive
    );


    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoVisionCommand(this._targetVision),
        new TurretPositionCommand(this._turret, -63),//-87
        new AutoIntakeDeploy(this._pneumatics),
        new InstantCommand(() -> _drive.resetOdometry(testPath.getInitialPose()))
      ),
      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          command,
          new InstantCommand(() -> _drive.stopModules())
        )
      ),
      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 4, this._targetVision, this._turret),  
        new IndexerCommand(this._indexer, -0.7, 4)    
      )
                      
    );
  }


  public Command testPathPlanner(){
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.FrameConstants.kDriveKinematics);

    // 2. Generate trajectory
    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", 4, 3);
    PathPlannerTrajectory testPath2 = PathPlanner.loadPath("Test Path 02", 4, 3);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      testPath,
      //swerveSubsystem::getPose,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      _trajectorySetup.xController,
      _trajectorySetup.yController,
      _trajectorySetup.thetaController,
      _drive::setModuleStates,
      _drive
    );


    PPSwerveControllerCommand command2 = new PPSwerveControllerCommand(
      testPath2,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      _trajectorySetup.xController,
      _trajectorySetup.yController,
      _trajectorySetup.thetaController,
      _drive::setModuleStates,
      _drive
    );

    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoVisionCommand(this._targetVision),
        new TurretPositionCommand(this._turret, -63),//-87
        new AutoIntakeDeploy(this._pneumatics),
        new InstantCommand(() -> _drive.resetOdometry(testPath.getInitialPose()))
      ),
      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          command,
          new InstantCommand(() -> _drive.stopModules())
        )
      ),
      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 4, this._targetVision, this._turret),  
        new IndexerCommand(this._indexer, -0.7, 4)    
      ),

      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          new TurretPositionCommand(this._turret, -160),//-83
          command2,
          new InstantCommand(() -> _drive.stopModules())
        )
      ),
      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 4, this._targetVision, this._turret),  
        new IndexerCommand(this._indexer, -0.7, 4)    
      )         
    );
  }

  public Command SlimMonkey4Ball(){
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.FrameConstants.kDriveKinematics);    

    // 2. Generate trajectory
    PathPlannerTrajectory testPath = PathPlanner.loadPath("Slim Monkey", 4, 3);
    PathPlannerTrajectory testPath2 = PathPlanner.loadPath("Slim Monkey2", 4, 3);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      testPath,
      //swerveSubsystem::getPose,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      _trajectorySetup.xController,
      _trajectorySetup.yController,
      _trajectorySetup.thetaController,
      _drive::setModuleStates,
      _drive
    );

    PPSwerveControllerCommand command2 = new PPSwerveControllerCommand(
      testPath2,
      //swerveSubsystem::getPose,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      _trajectorySetup.xController,
      _trajectorySetup.yController,
      _trajectorySetup.thetaController,
      _drive::setModuleStates,
      _drive
    );

    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoVisionCommand(this._targetVision),
        new TurretPositionCommand(this._turret, -100),//-87
        new AutoIntakeDeploy(this._pneumatics),
        new InstantCommand(() -> _drive.resetOdometry(testPath.getInitialPose()))
      ),
      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          command,
          new InstantCommand(() -> _drive.stopModules())
        )
      ),
      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 4, this._targetVision, this._turret),  
        new IndexerCommand(this._indexer, -0.7, 4)    
      ),
      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          new TurretPositionCommand(this._turret, -125),//-83
          command2,
          new InstantCommand(() -> _drive.stopModules())
        )
      ),
      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 4, this._targetVision, this._turret),  
        new IndexerCommand(this._indexer, -0.7, 4)    
      )
                    
    );
  }

  public Command testMonkey2BallAuto(){

    /*
            Generate the trajectory from the path created in PathPlanner
    */
    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", 4, 3);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      testPath,
      _drive::getPose,
      Constants.FrameConstants.kDriveKinematics,
      _trajectorySetup.xController,
      _trajectorySetup.yController,
      _trajectorySetup.thetaController,
      _drive::setModuleStates,
      _drive
    );


    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoVisionCommand(this._targetVision),
        new TurretPositionCommand(this._turret, -63),//-87
        new AutoIntakeDeploy(this._pneumatics),
        new InstantCommand(() -> _drive.resetOdometry(testPath.getInitialPose()))
      ),
      new ParallelCommandGroup(
        new IntakeCommand(this._intake, 1.0, 3.5),
        new SequentialCommandGroup(
          command,
          new InstantCommand(() -> _drive.stopModules())
        )
      ),
      new ParallelCommandGroup(
        new VisionShooterCommand(this._launcher, 4, this._targetVision, this._turret),  
        new IndexerCommand(this._indexer, -0.7, 4)    
      )              
    );
  }

  public class TrajectorySetup {
    public TrajectoryConfig trajectoryConfig;
    public PIDController xController;
    public PIDController yController;
    public PIDController thetaController;
    // public ProfiledPIDController thetaController;

    public TrajectorySetup(){
      /*
        Create trajectory settings
      */
      trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.FrameConstants.kDriveKinematics);

      /*
        Define the PID controllers for tracking trajectory
      */
      xController = new PIDController(AutoConstants.kPXController, 0, 0);
      yController = new PIDController(AutoConstants.kPYController, 0, 0);
      thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

      // thetaController = new ProfiledPIDController(
      //   AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
      // );
      // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

  }

}