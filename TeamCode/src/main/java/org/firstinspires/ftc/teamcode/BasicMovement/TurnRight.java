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
@Autonomous(name = "4: Turn Sequence", group = "Simple Auto")
public class TurnRight extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.4);

        Pose p0 = new Pose(0, 0, 0);
        follower.setStartingPose(p0);

        waitForStart();
        if (isStopRequested()) return;

        Pose p1 = new Pose(2, 0, Math.toRadians(-90));
        Path path1 = new Path(new BezierLine(p0, p1));
        path1.setLinearHeadingInterpolation(0, Math.toRadians(-90));
        follower.followPath(path1, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); }

        Pose p2 = new Pose(4, 0, Math.toRadians(-180));
        Path path2 = new Path(new BezierLine(p1, p2));
        path2.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180));
        follower.followPath(path2, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); }

        Pose p3 = new Pose(2, 0, Math.toRadians(-270));
        Path path3 = new Path(new BezierLine(p2, p3));
        path3.setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-270));
        follower.followPath(path3, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); }

        Pose p4 = new Pose(0, 0, Math.toRadians(-360));
        Path path4 = new Path(new BezierLine(p3, p4));
        path4.setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(-360));
        follower.followPath(path4, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); }

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Final Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
