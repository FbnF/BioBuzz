package org.firstinspires.ftc.teamcode.BiobuzzAuto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "0LeaveParkP2", group = "Autonomous")
public class ZeroLeaveParkP2 extends LinearOpMode {
    private Servo frontServo;

    private Follower follower;

    public static double startPoseX = 8.8966;
    public static double startPoseY = 77.6421;
    public static double startPoseRotation = 0;

    public static double parkPoseX = 12.5687;
    public static double parkPoseY = 123.8756;
    public static double parkPoseRotation = 0;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose parkPose = new Pose(parkPoseX, parkPoseY, Math.toRadians(parkPoseRotation));

    private PathChain driveToPark;

    public void buildPaths() {
        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    public Command autoRoutine() {

        return sequential(
                follow(follower, driveToPark, true),
                parallel(


                )
        );
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);

        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);
        frontServo = hardwareMap.get(Servo.class, "frontServo");
        telemetry.addLine("0LeaveParkP2 Ready.");
        telemetry.update();
        frontServo.setPosition(ShooterConfig.FRONT_SERVO_INIT_POS);

        waitForStart();

        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
