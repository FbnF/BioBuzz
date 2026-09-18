package org.firstinspires.ftc.teamcode.NewTeleOp.Services;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

// Status light logic
public class TurretService {

    private static TurretService instance = null;

    // LED positions

    //Angle in radians outside of what we can turn to.
    private static final double max = 5.14;
    private static final double min = 0;
    private static final double radius = 2.23607;
    public double GetRotationAngle(double targetX, double targetY){
        double angleOfHive =  Math.atan(targetX/targetY);
        if(angleOfHive  <= 5.14 && angleOfHive >= 0){
            double Vx = Math.cos(angleOfHive) * 10;
            double Vy = Math.sin(angleOfHive) * 10;
            double V = Math.sqrt(Math.pow(Vx, 2)+Math.pow(Vy, 2));
            double X = Vx/Math.abs(V) * 2.23607;
            double Y = Vy/Math.abs(V) * 2.23607;
            return Math.atan(X/Y);


        } else{
            return(0.0);
        }

    }
}