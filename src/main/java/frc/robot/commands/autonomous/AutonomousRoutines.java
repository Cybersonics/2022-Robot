package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.NavXGyro;
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

    public AutonomousRoutines() {
    }

    public Command DoNothing() {
        return new DoNothingCommand();
    }
}