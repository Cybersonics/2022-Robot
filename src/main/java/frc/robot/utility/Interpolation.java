package frc.robot.utility;

import frc.robot.Constants;

public class Interpolation {    
    public static double getReference(double distance) {
        double minDist = 0.0, minRPM = 0.0;
        double maxDist = 0.0, maxRPM = 0.0;
        double[][] setPoints = Constants.distanceReference;

        if (distance <= Constants.MIN_REFERENCE[0]) {
            minDist = distance;
            minRPM = Constants.MIN_REFERENCE[1];
            maxDist = Constants.MIN_REFERENCE[0];
            maxRPM = Constants.MIN_REFERENCE[1];
        } else if (distance >= Constants.MAX_REFERENCE[0]) {
            minDist = Constants.MAX_REFERENCE[0];
            minRPM = Constants.MAX_REFERENCE[1];
            maxDist = distance;
            maxRPM = Constants.MAX_REFERENCE[1];
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

        return (minRPM + (distance - minDist) * (dif) / (maxDist - minDist));
    }
}
