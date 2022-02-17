// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.AutonomousRoutines;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.NavXGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  public static Drive _drive = Drive.getInstance(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH);
  public static Climber _climber = Climber.getInstance();
  public static Indexer _indexer = Indexer.getInstance();
  public static Intake _intake = Intake.getInstance();
  public static Launcher _launcher = Launcher.getInstance();
  public static NavXGyro _gyro = NavXGyro.getInstance();

  //Commands

  // Controllers
  public XboxController driveController;

  // A chooser for autonomous commands
  private final AutonomousRoutines _autonRoutines = new AutonomousRoutines();
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  public RobotContainer() {    
    // Set up auton selector
    m_chooser.setDefaultOption("Do Nothing", _autonRoutines.DoNothing());

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
    // Configure the button bindings
    configureButtonBindings();
    
    driveController = new XboxController(Constants.XBOX_CONTROLLER);

    CommandScheduler.getInstance()
    .setDefaultCommand(_drive, 
      new DriveCommand(_drive, driveController)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
