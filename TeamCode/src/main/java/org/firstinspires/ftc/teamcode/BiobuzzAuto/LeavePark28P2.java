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
@Autonomous(name = "28P2", group = "Autonomous")
public class LeavePark28P2 extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private Servo blockServo;
    private Servo frontServo;
    private DistanceSensor rangeSensor;

    private final double shooter_Vel = 1165.0;

    public static double startPoseX = 8.9778;
    public static double startPoseY = 62.6064;
    public static double startPoseRotation = 0;

    public static double path1Control1X = 27.2473;
    public static double path1Control1Y = 52.8683;
    public static double path1Control1Rotation = 0;

    public static double shootPoseX = 58.817;
    public static double shootPoseY = 11.7305;
    public static double shootPoseRotation = 90;

    public static double point2Control1X = 30.0919;
    public static double point2Control1Y = 48.8285;
    public static double point2Control1Rotation = 0;

    public static double parkPoseX = 16.1784;
    public static double parkPoseY = 93.4541;
    public static double parkPoseRotation = 290;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose path1Control1 = new Pose(path1Control1X, path1Control1Y, Math.toRadians(path1Control1Rotation));
    private final Pose shootPose = new Pose(shootPoseX, shootPoseY, Math.toRadians(shootPoseRotation));
    private final Pose point2Control1 = new Pose(point2Control1X, point2Control1Y, Math.toRadians(point2Control1Rotation));
    private final Pose parkPose = new Pose(parkPoseX, parkPoseY, Math.toRadians(parkPoseRotation));

    private PathChain driveToShoot, driveToPark;

    public void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, path1Control1, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, point2Control1, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
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
                        instant(() -> sideServo.setPower(1.0))
                ),
                waitMs(4000),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.26);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    public Command autoRoutine() {
        return sequential(
                parallel(
                        follow(follower, driveToShoot, true),
                        instant(() -> shooter.setVelocity(shooter_Vel))
                ),
                combinedShootLogic(),
                follow(follower, driveToPark, true),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.26);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
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
        blockServo = hardwareMap.get(Servo.class, HardwareConfig.BLOCK_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));

        blockServo.setPosition(0.26);
        frontServo.setPosition(ShooterConfig.FRONT_SERVO_INIT_POS);

        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("RED28LeavePark Ready.");
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