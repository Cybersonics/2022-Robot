package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher 
{
    private final double MAX_SHOOTER_RATE = 1.0;
    public double PivotRate = 0.5
    public CANSparkMax L_Launcher;
    public CANSparkMax R_Launcher;

    //calculate launch speed
    public void calculatedLaunch(double speed) {
        L_Launcher.set(speed);
        R_Launcher.set(speed);
      }

    
}
