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

@Autonomous(name = "BigTriRED9Clear", group = "Autonomous")
public class BigTriRed9Clear extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Latest visualizer poses
    private final Pose startPose = new Pose(108.582, 132.904, Math.toRadians(270));
    private final Pose scorePose1 = new Pose(92.510, 91.727, Math.toRadians(45));
    private final Pose intake1Start = new Pose(95.014, 81.243, Math.toRadians(0));
    private final Pose intake1End = new Pose(123, 80.865, Math.toRadians(0));
    private final Pose clearStart = new Pose(115, 75, Math.toRadians(90));
    private final Pose clearEnd = new Pose(126, 73, Math.toRadians(90));
    private final Pose scorePose2 = new Pose(82.728, 101.335, Math.toRadians(37));
    private final Pose intake2Start = new Pose(95.104, 59, Math.toRadians(7));
    private final Pose intake2End = new Pose(128, 59, Math.toRadians(7));
    private final Pose scorePose3 = new Pose(82.728, 101.335, Math.toRadians(35));

    private PathChain driveToShoot1, driveToIntake1, driveToClear, driveToShoot2, goToRack2, driveThroughRack2, driveToShoot3;

    // Building paths with global deceleration
    public void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intake1Start))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intake1Start.getHeading())
                .addPath(new BezierLine(intake1Start, intake1End))
                .setConstantHeadingInterpolation(intake1Start.getHeading())
                .setGlobalDeceleration()
                .build();
        driveToClear = follower.pathBuilder()
                .addPath(new BezierLine(intake1End, clearStart))
                .setLinearHeadingInterpolation(intake1End.getHeading(), clearStart.getHeading())
                .addPath(new BezierLine(clearStart, clearEnd))
                .setConstantHeadingInterpolation(clearStart.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(clearEnd, scorePose2))
                .setLinearHeadingInterpolation(intake1End.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();

        // Split paths for Rack 2 to prevent right-bias corner cutting
        goToRack2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, intake2Start))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), intake2Start.getHeading())
                .setGlobalDeceleration()
                .build();

        driveThroughRack2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Start, intake2End))
                .setConstantHeadingInterpolation(intake2Start.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(intake2End, scorePose3))
                .setLinearHeadingInterpolation(intake2End.getHeading(), scorePose3.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    // Shooter and reload logic
    public Command combinedShootLogic() {
        return sequential(
                // Small beat since we spin while driving
                waitMs(250),

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

                // Idle at half speed
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(600); 
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    // Complete auto routine
    public Command autoRoutine() {
        return sequential(
                // Shot 1 + Early Start
                parallel(
                        follow(follower, driveToShoot1, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Pickup 1 (at 0.6 power)
                instant(() -> follower.setMaxPower(0.6)),
                parallel(
                        follow(follower, driveToIntake1, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),
                //clear rack
                follow(follower, driveToClear, true),

                // Shot 2 + Early Start
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Pickup 2 - Forces snap to start position to fix overshooting right
                follow(follower, goToRack2, true), 
                
                instant(() -> follower.setMaxPower(0.6)),
                parallel(
                        follow(follower, driveThroughRack2, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),



                // Shot 3 + Early Start
                parallel(
                        follow(follower, driveToShoot3, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Shutdown
                instant(() -> shooter.setVelocity(0))
        );
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareConfig.SHOOTER_MOTOR);
        feed = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));

        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("BigTriRed9 Ready.");
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
