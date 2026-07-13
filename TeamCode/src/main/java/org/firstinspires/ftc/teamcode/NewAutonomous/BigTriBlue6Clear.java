package org.firstinspires.ftc.teamcode.NewAutonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

@Autonomous(name = "BigTriBLUE6Clear", group = "Autonomous")
public class BigTriBlue6Clear extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Latest Poses from visualizer
    private final Pose startPose = new Pose(33.814, 133.428, Math.toRadians(270));
    private final Pose scorePose1 = new Pose(49.362, 91.028, Math.toRadians(135));
    private final Pose intakeStart = new Pose(46.806, 83.648, Math.toRadians(180));
    private final Pose intakeEnd = new Pose(14.537, 84.082, Math.toRadians(180));
    private final Pose clearStart = new Pose(26.796, 70.959, Math.toRadians(270));
    private final Pose clearEnd = new Pose(12.486, 70.337, Math.toRadians(270));
    private final Pose scorePose2 = new Pose(58.271, 100.461, Math.toRadians(145));

    private PathChain driveToShoot1, driveToIntake, driveToClear, driveToShoot2;

    // Build paths
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

    // Shoot and reload
    public Command combinedShootLogic() {
        return sequential(
                // Spin up (short range)
                instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT)),

                // Stabilize
                waitMs((long)(ShooterConfig.START_WAIT_TIME * 1000)),

                // Feed
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER)),

                // Wait for sensor clear
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),

                // Reload
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // 5s settle wait
                waitMs(5000),

                // Power down
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    // Main routine
    public Command autoRoutine() {
        return sequential(
                // Score preload
                follow(follower, driveToShoot1, true),
                combinedShootLogic(),

                // Pickup (50% power)
                instant(() -> follower.setMaxPower(0.5)),
                parallel(
                        follow(follower, driveToIntake, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),

                // Clear the bar
                follow(follower, driveToClear, true),

                // Stop intake for score move
                instant(() -> {
                    intake.setPower(0);
                    sideServo.setPower(0);
                }),

                // Score second ball
                follow(follower, driveToShoot2, true),
                combinedShootLogic()
        );
    }

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

        telemetry.addLine("BigTriBlue6Clear Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Schedule auto
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Status
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}