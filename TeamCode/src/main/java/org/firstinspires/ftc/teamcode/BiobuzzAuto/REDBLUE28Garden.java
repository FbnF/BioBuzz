package org.firstinspires.ftc.teamcode.BiobuzzAuto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "REDBLUE28Garden", group = "Autonomous")
public class REDBLUE28Garden extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private CRServo frontServo;
    private Servo blockServo;
    private DistanceSensor rangeSensor;

    private final double shooter_Vel = 1165.0;

    public static double startPoseX = 56.2167;
    public static double startPoseY = 9.0835;
    public static double startPoseRotation = 90;

    public static double path1X = 44.0819;
    public static double path1Y = 19.379;
    public static double path1Rotation = 180;

    public static double point2X = 29.6792;
    public static double point2Y = 9.4127;
    public static double point2Rotation = 180;

    public static double point3X = 10.7726;
    public static double point3Y = 9.2443;
    public static double point3Rotation = 180;

    public static double point4X = 34.9456;
    public static double point4Y = 104.8055;
    public static double point4Rotation = 250;

    public static double point5Control1X = 59.4334;
    public static double point5Control1Y = 120.2067;
    public static double point5Control1Rotation = 0;

    public static double point5X = 61.9617;
    public static double point5Y = 132.0574;
    public static double point5Rotation = 270;

    private final Pose start = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose p1 = new Pose(path1X, path1Y, Math.toRadians(path1Rotation));
    private final Pose p2 = new Pose(point2X, point2Y, Math.toRadians(point2Rotation));
    private final Pose p3 = new Pose(point3X, point3Y, Math.toRadians(point3Rotation));
    private final Pose p4 = new Pose(point4X, point4Y, Math.toRadians(point4Rotation));
    private final Pose p5Control1 = new Pose(point5Control1X, point5Control1Y, Math.toRadians(point5Control1Rotation));
    private final Pose p5 = new Pose(point5X, point5Y, Math.toRadians(point5Rotation));

    private PathChain path1Chain, path2Chain, path3Chain, path4Chain, path5Chain;

    public void buildPaths() {
        path1Chain = follower.pathBuilder()
                .addPath(new BezierLine(start, p1))
                .setLinearHeadingInterpolation(start.getHeading(), p1.getHeading())
                .setGlobalDeceleration()
                .build();

        path2Chain = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .setGlobalDeceleration()
                .build();

        path3Chain = follower.pathBuilder()
                .addPath(new BezierLine(p2, p3))
                .setLinearHeadingInterpolation(p2.getHeading(), p3.getHeading())
                .setGlobalDeceleration()
                .build();

        path4Chain = follower.pathBuilder()
                .addPath(new BezierLine(p3, p4))
                .setLinearHeadingInterpolation(p3.getHeading(), p4.getHeading())
                .setGlobalDeceleration()
                .build();

        path5Chain = follower.pathBuilder()
                .addPath(new BezierCurve(p4, p5Control1, p5))
                .setLinearHeadingInterpolation(p4.getHeading(), p5.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    public Command combinedShootLogic() {
        return sequential(
                waitUntil(() -> Math.abs(shooter.getVelocity() - shooter_Vel) < ShooterConfig.TPS_TOL),
                parallel(
                        instant(() -> feed.setPower(-0.9)),
                        instant(() -> blockServo.setPosition(0.38)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> frontServo.setPower(-1.0))
                ),
                waitMs(4000),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.15);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                    frontServo.setPower(0);
                })
        );
    }

    public Command autoRoutine() {
        return sequential(
                instant(() -> follower.setMaxPower(0.6)),
                follow(follower, path1Chain, true),
                follow(follower, path2Chain, true),
                follow(follower, path3Chain, true),
                follow(follower, path4Chain, true),
                parallel(
                        follow(follower, path5Chain, true),
                        instant(() -> shooter.setVelocity(shooter_Vel))
                ),
                instant(() -> follower.setMaxPower(1.0)),
                combinedShootLogic(),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.15);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                    frontServo.setPower(0);
                })
        );
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareConfig.SHOOTER_MOTOR);
        feed = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        frontServo = hardwareMap.get(CRServo.class, HardwareConfig.FRONT_SERVO);
        blockServo = hardwareMap.get(Servo.class, HardwareConfig.BLOCK_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));

        blockServo.setPosition(0.15);

        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(start);

        telemetry.addLine("REDBLUE28Garden Ready.");
        telemetry.update();

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
