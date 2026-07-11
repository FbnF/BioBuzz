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

@Autonomous(name = "BigTriRED9", group = "Autonomous")
public class BigTriRed9 extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Poses from visualizer (Red Side)
    private final Pose startPose = new Pose(108.582, 132.904, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(92.510, 91.727, Math.toRadians(45));
    private final Pose intake1Start = new Pose(95.014, 81.243, Math.toRadians(0));
    private final Pose intake1End = new Pose(123.016, 80.865, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(82.728, 101.335, Math.toRadians(35));
    private final Pose intake2Start = new Pose(96.152, 59.110, Math.toRadians(0));
    private final Pose intake2End = new Pose(129.796, 58.696, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(82.728, 101.335, Math.toRadians(35));

    private PathChain driveToShoot1, driveToIntake1, driveToShoot2, driveToIntake2, driveToShoot3;

    // Paths logic with global deceleration
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

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1End, scorePose2))
                .setLinearHeadingInterpolation(intake1End.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, intake2Start))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), intake2Start.getHeading())
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
                // Settle beat after drive
                waitMs(250),

                // Engage feed using config (currently -1.0)
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER)),

                // Wait for ball to fire
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),

                // Pull in next ball
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // Wait for ball to settle (4.2s target)
                waitMs(4200),

                // Idle flywheel at half speed
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(600);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    // Main routine
    public Command autoRoutine() {
        return sequential(
                // Initial preload
                parallel(
                        follow(follower, driveToShoot1, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // First floor pickup (at 0.6 power)
                instant(() -> follower.setMaxPower(0.6)),
                parallel(
                        follow(follower, driveToIntake1, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),

                // Second shot
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Second floor pickup (at 0.6 power)
                instant(() -> follower.setMaxPower(0.6)),
                parallel(
                        follow(follower, driveToIntake2, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),

                // Third shot
                parallel(
                        follow(follower, driveToShoot3, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Final match shutdown
                instant(() -> shooter.setVelocity(0))
        );
    }

    @Override
    public void runOpMode() {
        // Hardware Mapping
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

            // Monitoring
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
