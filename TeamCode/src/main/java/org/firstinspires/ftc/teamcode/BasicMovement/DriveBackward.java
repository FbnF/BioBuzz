package org.firstinspires.ftc.teamcode.BasicMovement;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "DriveBackward", group = "Autonomous")
public class DriveBackward extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        
        Pose startPose = new Pose(0, 0, 0);
        Pose endPose = new Pose(-48, 0, 0);

        follower.setStartingPose(startPose);
        
        Path driveBackward = new Path(new BezierLine(startPose, endPose));
        
        driveBackward.setConstantHeadingInterpolation(0);

        telemetry.addLine("Backward 48 in");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        
        follower.followPath(driveBackward);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        telemetry.addLine("Path Complete!");
        telemetry.update();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Final X", follower.getPose().getX());
            telemetry.addData("Final Y", follower.getPose().getY());
            telemetry.addData("Final Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
