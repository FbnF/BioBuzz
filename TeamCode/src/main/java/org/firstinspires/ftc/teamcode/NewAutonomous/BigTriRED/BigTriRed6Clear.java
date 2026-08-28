package org.firstinspires.ftc.teamcode.NewAutonomous.BigTriRED;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "BigTriRED6Clear", group = "Autonomous")
public class BigTriRed6Clear extends LinearOpMode {

    //-------------------- Hardware & Follower Constants --------------------
    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;



    //-------------------- POSES --------------------
    public static double startPoseX = 108.582;
    public static double startPoseY = 132.904;
    public static double startPoseRotation = 270;

    public static double scorePose1X = 92.510;
    public static double scorePose1Y = 91.727;
    public static double scorePose1Rotation = 45;

    public static double intakeStartX = 95.014;
    public static double intakeStartY = 81.243;
    public static double intakeStartRotation = 0;

    public static double intakeEndX = 123;
    public static double intakeEndY = 80.865;
    public static double intakeEndRotation = 0;

    public static double clearStartX = 121.921;
    public static double clearStartY = 74.278;
    public static double clearStartRotation = 90;

    public static double clearEndX = 129.0;
    public static double clearEndY = 73.918;
    public static double clearEndRotation = 90;

    public static double scorePose2X = 82.728;
    public static double scorePose2Y = 101.335;
    public static double scorePose2Rotation = 34;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose scorePose1 = new Pose(scorePose1X, scorePose1Y, Math.toRadians(scorePose1Rotation));
    private final Pose intakeStart = new Pose(intakeStartX, intakeStartY, Math.toRadians(intakeStartRotation));
    private final Pose intakeEnd = new Pose(intakeEndX, intakeEndY, Math.toRadians(intakeEndRotation));
    private final Pose clearStart = new Pose(clearStartX, clearStartY, Math.toRadians(clearStartRotation));
    private final Pose clearEnd = new Pose(clearEndX, clearEndY, Math.toRadians(clearEndRotation));
    private final Pose scorePose2 = new Pose(scorePose2X, scorePose2Y, Math.toRadians(scorePose2Rotation));



    //-------------------- Defined Paths --------------------
    private PathChain driveToShoot1, driveToIntake, driveToClear, driveToShoot2;

    public void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToIntake = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intakeStart))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakeStart.getHeading())
                .addPath(new BezierLine(intakeStart, intakeEnd))
                .setConstantHeadingInterpolation(intakeStart.getHeading())
                .setGlobalDeceleration()
                .build();

        // Extra path to let the balls out
        driveToClear = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, clearStart))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), clearStart.getHeading())
                .addPath(new BezierLine(clearStart, clearEnd))
                .setConstantHeadingInterpolation(clearStart.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(clearEnd, scorePose2))
                .setLinearHeadingInterpolation(clearEnd.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();
    }



    //-------------------- Shooter & Reload Logic --------------------
    public Command combinedShootLogic() {
        return sequential(
                // Wait until the flywheel is at target speed
                waitUntil(() -> Math.abs(shooter.getVelocity() - ShooterConfig.SHOOTER_VEL_SHORT) < ShooterConfig.TPS_TOL),

                // Engagement using config SIDE_POWER (-1.0)
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER)),

                // Wait for fire
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),

                // Start reload
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // 4.2s settled wait
                waitMs(4200),

                // Idle (Shut off for intake safety)
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }



    //-------------------- Auto Routine --------------------
    public Command autoRoutine() {
        return sequential(
                // Score 1
                parallel(
                        follow(follower, driveToShoot1, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Intake (50% power)
                instant(() -> follower.setMaxPower(0.5)),
                parallel(
                        follow(follower, driveToIntake, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),

                // Clear the area
                follow(follower, driveToClear, true).raceWith(waitMs(2000)),
                waitMs(250),

                // Stop intake for score move
                instant(() -> {
                    intake.setPower(0);
                    sideServo.setPower(0);
                }),

                // Score 2
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),
                // Power down
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }



    //-------------------- OpMode Setup --------------------
    @Override
    public void runOpMode() {
        // Init hardware
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareConfig.SHOOTER_MOTOR);
        feed = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        // Subsystem setup
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));
        
        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("BigTriRed6Clear Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Schedule auto
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Telemetry
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
