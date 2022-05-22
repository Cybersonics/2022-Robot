package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Interpolation {    
    public static double getRPMReference(double distance) {
        double minDist = 0.0, minRPM = 0.0;
        double maxDist = 0.0, maxRPM = 0.0;
        double[][] setPoints = Constants.distanceReference;

        if (distance <= Constants.MIN_RPM_REFERENCE[0]) {
            minDist = distance;
            minRPM = Constants.MIN_RPM_REFERENCE[1];
            maxDist = Constants.MIN_RPM_REFERENCE[0];
            maxRPM = Constants.MIN_RPM_REFERENCE[1];
        } else if (distance >= Constants.MAX_RPM_REFERENCE[0]) {
            minDist = Constants.MAX_RPM_REFERENCE[0];
            minRPM = Constants.MAX_RPM_REFERENCE[1];
            maxDist = distance;
            maxRPM = Constants.MAX_RPM_REFERENCE[1];
        } else {
            for(int i = 0; i < setPoints.length; i++) {
                if(setPoints[i][0] >= distance) {
                    maxRPM = setPoints[i][1];
                    maxDist = setPoints[i][0];
                } else if (setPoints[i][0] <= distance) {
                    minRPM = setPoints[i][1];
                    minDist = setPoints[i][0];
                }
            }
        }
        double dif = maxRPM - minRPM;
        
        // y = 0.0529x2 - 5.3439x + 2907.7
        // return 0.0529 * Math.Pow(distance,2) - 5.3439 * distance + 2907.7;
        
        double bestFitValue = Math.round(0.0529 * Math.pow(distance,2) - 5.3439 * distance + 2907.7);
        double interpolationValue = Math.round(minRPM + (distance - minDist) * (dif) / (maxDist - minDist));
        SmartDashboard.putNumber("RPM Best Fit", bestFitValue);
        SmartDashboard.putNumber("RPM Interpolation", interpolationValue);
        return bestFitValue;
    }

    public static double getAngleReference(double distance) {
        double minDist = 0.0, minAngle = 0.0;
        double maxDist = 0.0, maxAngle = 0.0;
        double[][] setPoints = Constants.angleReference;
        
        if (distance <= Constants.MIN_ANGLE_REFERENCE[0]) {
            minDist = distance;
            minAngle = Constants.MIN_ANGLE_REFERENCE[1];
            maxDist = Constants.MIN_ANGLE_REFERENCE[0];
            maxAngle = Constants.MIN_ANGLE_REFERENCE[1];
        } else if (distance >= Constants.MAX_ANGLE_REFERENCE[0]) {
            minDist = Constants.MAX_ANGLE_REFERENCE[0];
            minAngle = Constants.MAX_ANGLE_REFERENCE[1];
            maxDist = distance;
            maxAngle = Constants.MAX_ANGLE_REFERENCE[1];
        } else {
            for(int i = 0; i < setPoints.length; i++) {
                if(setPoints[i][0] >= distance) {
                    maxAngle = setPoints[i][1];
                    maxDist = setPoints[i][0];
                } else if (setPoints[i][0] <= distance) {
                    minAngle = setPoints[i][1];
                    minDist = setPoints[i][0];
                }
            }
        }
        double dif = maxAngle - minAngle;
        
        // y = y = 0.0021x2 - 0.0519x - 15.741
        double bestFitValue = 0.0021 * Math.pow(distance, 2) - 0.0519 * distance - 15.741;
        double interpolationValue = minAngle + (distance - minDist) * (dif) / (maxDist - minDist);

        SmartDashboard.putNumber("Angle Best Fit", bestFitValue);
        SmartDashboard.putNumber("Angle Interpolation", interpolationValue);
        return bestFitValue;
    }
}
