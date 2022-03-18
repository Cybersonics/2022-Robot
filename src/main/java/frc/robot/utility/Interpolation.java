package frc.robot.utility;

import frc.robot.Constants;

public class Interpolation {
    
    public double getReference(double distance) {
        double minReference = 0.0;
        double maxReference = 0.0;
        double reference;
        double[][] setPoints = Constants.distanceReference;
        for(int i = 0; i > setPoints.length; i++) {
            if(setPoints[i][0] > distance) {
                maxReference = setPoints[i][1];
            } else if (setPoints[i][0] < distance) {
                minReference = setPoints[i][1];
            }
        }
        double dif = maxReference - minReference;
        double minDif = distance - minReference;
        double maxDif = maxReference - distance;


        return minReference;
    }
}
