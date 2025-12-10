package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class AngleOptimize {
    static double tau = 2*Math.PI;
    public static Rotation2d optimizeAngle(Rotation2d currentAngle, Rotation2d targetAngle){
        double currentRadians = currentAngle.getRadians();

        double closestFullRotation = Math.floor(Math.abs(currentRadians/tau))*(currentRadians < 0 ? -1 : 1) * tau;
        double currentOptimalAngle = targetAngle.getRadians() + closestFullRotation - currentRadians;
        double[] potentialNewAngles = new double[] {
            currentOptimalAngle,
            currentOptimalAngle - tau,
            currentOptimalAngle + tau
        };

        double deltaAngle = tau;
        for(double potentialAngle:potentialNewAngles){
            if(Math.abs(deltaAngle) > Math.abs(potentialAngle)){
                deltaAngle = potentialAngle;
            }
        }

        return new Rotation2d(deltaAngle+currentRadians);
    }

    public static double armAccountForSillyEncoder(double startupPosition){
        final double kArmRealZero = Constants.Arm.kArmRealZero;

        double realPosition;
        if(startupPosition < kArmRealZero){
            realPosition = tau + startupPosition;
        }else{
            realPosition=startupPosition;
        }

        return realPosition;
    }
}
