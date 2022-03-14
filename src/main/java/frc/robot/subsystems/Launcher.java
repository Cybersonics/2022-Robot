package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private final double MAX_SHOOTER_RATE = 0.75;//0.85
  public CANSparkMax _leftMotor;
  public CANSparkMax _rightMotor;
  private static Launcher instance;

  private Launcher() {
    CommandScheduler.getInstance().registerSubsystem(this);
    setupRightMotor();
    setupLeftMotor();
  }

  // Public Methods
  public static Launcher getInstance() {
    if (instance == null) {
      instance = new Launcher();
    }
    return instance;
  }

  public void calculatedLaunch(double speed) {
    _rightMotor.set(speed);
    _leftMotor.set(speed);
  }

  public void start() {
    calculatedLaunch(MAX_SHOOTER_RATE);
  }

  public void stop() {
    calculatedLaunch(0);
  }

  // Private Methods
  private void setupRightMotor() {
    _rightMotor = new CANSparkMax(Constants.R_Launcher, MotorType.kBrushless);
    _rightMotor.restoreFactoryDefaults();
    _rightMotor.setIdleMode(IdleMode.kCoast);
  }

  private void setupLeftMotor() {
    _leftMotor = new CANSparkMax(Constants.L_Launcher, MotorType.kBrushless);
    _leftMotor.restoreFactoryDefaults();
    _leftMotor.setIdleMode(IdleMode.kCoast);
    _leftMotor.setInverted(true);
  }
}
