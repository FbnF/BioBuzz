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
@Autonomous(name = "REDBLUE28WaitStart", group = "Autonomous")
public class REDBLUE28WaitStart extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private CRServo intakeHelper;
    private Servo frontServo;
    private Servo blockServo;
    private DistanceSensor rangeSensor;

    private final double shooter_Vel = 1165.0;

    public static double startPoseX = 62.5008;
    public static double startPoseY = 133.2481;
    public static double startPoseRotation = 90;

    public static double path1StartX = 62.5008;
    public static double path1StartY = 133.2481;
    public static double path1StartRotation = 270;

    public static double path1Control1X = 59.9119;
    public static double path1Control1Y = 115.8859;
    public static double path1Control1Rotation = 0;

    public static double parkPoseX = 13.9617;
    public static double parkPoseY = 115.0926;
    public static double parkPoseRotation = 290;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose path1StartPose = new Pose(path1StartX, path1StartY, Math.toRadians(path1StartRotation));
    private final Pose path1Control1 = new Pose(path1Control1X, path1Control1Y, Math.toRadians(path1Control1Rotation));
    private final Pose parkPose = new Pose(parkPoseX, parkPoseY, Math.toRadians(parkPoseRotation));

    private PathChain path1Chain;

    public void buildPaths() {
        path1Chain = follower.pathBuilder()
                .addPath(new BezierCurve(path1StartPose, path1Control1, parkPose))
                .setLinearHeadingInterpolation(path1StartPose.getHeading(), parkPose.getHeading())
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
                        instant(() -> intakeHelper.setPower(-1.0))
                ),
                waitMs(4000),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.15);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                    intakeHelper.setPower(0);
                })
        );
    }

    public Command autoRoutine() {
        return sequential(
                parallel(
                        waitMs(8000),
                        instant(() -> shooter.setVelocity(shooter_Vel)),
                        instant(() -> blockServo.setPosition(0.15))
                ),
                combinedShootLogic(),
                follow(follower, path1Chain, true),
                instant(() -> {
                    feed.setPower(0);
                    blockServo.setPosition(0.15);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                    intakeHelper.setPower(0);
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
        frontServo = hardwareMap.get(Servo.class, HardwareConfig.FRONT_SERVO);
        blockServo = hardwareMap.get(Servo.class, HardwareConfig.BLOCK_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));

        blockServo.setPosition(0.15);
        frontServo.setPosition(0.28);
        intakeHelper.setPower(0.0);

        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("REDBLUE28WaitStart Ready.");
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
