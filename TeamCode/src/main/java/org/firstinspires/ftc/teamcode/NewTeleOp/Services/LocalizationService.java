package org.firstinspires.ftc.teamcode.NewTeleOp.Services;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LocalizationService {

    public static Pose superPose(Limelight3A limelight3A, Follower follower, double alpha) {
        double robotYaw = follower.getHeading();
        limelight3A.updateRobotOrientation(robotYaw);

        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        LLResult result = limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                Pose visionPose = MT2ToPedro(botpose_mt2);
                double visionX = visionPose.getX();
                double visionY = visionPose.getY();
                double visionHeading = visionPose.getHeading();
                currentX = currentX + alpha * (visionX - currentX);
                currentY = currentY + alpha * (visionY - currentY);

                double headingError = angleWrap(visionHeading - currentHeading);
                currentHeading = angleWrap(currentHeading + alpha * headingError);
            }
        }

        Pose finalPose = new Pose(currentX, currentY, currentHeading);
        follower.setPose(finalPose);
        return finalPose;
    }

    private static Pose MT2ToPedro(Pose3D MegaTag2) {
        double Px = MegaTag2.getPosition().y + 72;
        double Py = 72 - MegaTag2.getPosition().x;
        double Ph = Math.toRadians(MegaTag2.getOrientation().getYaw() + 90);
        return new Pose(Px, Py, Ph);
    }

    private static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians <= -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}