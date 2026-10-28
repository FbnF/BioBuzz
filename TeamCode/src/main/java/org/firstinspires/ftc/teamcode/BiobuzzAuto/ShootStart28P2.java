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
@Autonomous(name = "28P1", group = "Autonomous")
public class ShootStart28P2 extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private CRServo intakeHelper;
    private CRServo flowerServo;
    private Servo frontServo;
    private Servo blockServo;
    private DistanceSensor rangeSensor;

    private final double shooter_Vel = 1165.0;

    public static double startX = 56.0;
    public static double startY = 9.7335;
    public static double startRotation = 90;

    public static double point1X = 56.1156;
    public static double point1Y = 9.0743;
    public static double point1Rotation = 90;

    public static double path1StartX = 56.1156;
    public static double path1StartY = 9.0743;
    public static double path1StartRotation = 90;

    public static double path1Control1X = 27.6424;
    public static double path1Control1Y = 53.3308;
    public static double path1Control1Rotation = 0;

    public static double path1X = 25.4086;
    public static double path1Y = 94.6335;
    public static double path1Rotation = 290;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(startRotation));
    private final Pose point1Pose = new Pose(point1X, point1Y, Math.toRadians(point1Rotation));
    private final Pose path1StartPose = new Pose(path1StartX, path1StartY, Math.toRadians(path1StartRotation));
    private final Pose path1Control1 = new Pose(path1Control1X, path1Control1Y, Math.toRadians(path1Control1Rotation));
    private final Pose path1Pose = new Pose(path1X, path1Y, Math.toRadians(path1Rotation));

    private PathChain linePath, curvePath;

    public void buildPaths() {
        linePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, point1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), point1Pose.getHeading())
                .setGlobalDeceleration()
                .build();

        curvePath = follower.pathBuilder()
                .addPath(new BezierCurve(path1StartPose, path1Control1, path1Pose))
                .setLinearHeadingInterpolation(path1StartPose.getHeading(), path1Pose.getHeading())
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
                        instant(() -> intakeHelper.setPower(ShooterConfig.INTAKE_HELPER_INTAKE_POWER)),
                        instant(() -> frontServo.setPosition(ShooterConfig.FRONT_SERVO_FLOWER_POS)),
                        instant(() -> flowerServo.setPower(1.0))
                ),
                waitMs(4000),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.26);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                    intakeHelper.setPower(0);
                    flowerServo.setPower(0);
                    frontServo.setPosition(ShooterConfig.FRONT_SERVO_INIT_POS);
                })
        );
    }

    public Command autoRoutine() {
        return sequential(
                instant(() -> shooter.setVelocity(shooter_Vel)),
                combinedShootLogic(),
                follow(follower, linePath, true),
                follow(follower, curvePath, true),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.26);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                    intakeHelper.setPower(0);
                    flowerServo.setPower(0);
                    frontServo.setPosition(ShooterConfig.FRONT_SERVO_INIT_POS);
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
        intakeHelper = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_HELPER);
        flowerServo = hardwareMap.get(CRServo.class, HardwareConfig.FLOWER_SERVO);
        frontServo = hardwareMap.get(Servo.class, HardwareConfig.FRONT_SERVO);
        blockServo = hardwareMap.get(Servo.class, HardwareConfig.BLOCK_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));

        blockServo.setPosition(0.26);
        frontServo.setPosition(ShooterConfig.FRONT_SERVO_INIT_POS);
        intakeHelper.setPower(0.0);
        flowerServo.setPower(0.0);

        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("28P2 Ready.");
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
