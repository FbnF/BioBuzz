package org.firstinspires.ftc.teamcode.NewAutonomous.SmallTriangle;

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
@Autonomous(name = "SmallTriRed", group = "Autonomous")
public class SmallTriRed extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Latest Poses from the Visualizer
    public static double startPoseX = 86.298;
    public static double startPoseY = 8.385;
    public static double startPoseRotation = 90;

    public static double scorePose1X = 82.378;
    public static double scorePose1Y = 18.531;
    public static double scorePose1Rotation = 63;

    public static double intakeStartX = 92.326;
    public static double intakeStartY = 33.535;
    public static double intakeStartRotation = 0;

    public static double intakeEndX = 127.525;
    public static double intakeEndY = 33.366;
    public static double intakeEndRotation = 0;

    public static double scorePose2X = 82.378;
    public static double scorePose2Y = 18.531;
    public static double scorePose2Rotation = 63;

    public static double parkPoseX = 105.253;
    public static double parkPoseY = 16.638;
    public static double parkPoseRotation = 90;

    // Latest Poses from the Visualizer
    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose scorePose1 = new Pose(scorePose1X, scorePose1Y, Math.toRadians(scorePose1Rotation));
    private final Pose intakeStart = new Pose(intakeStartX, intakeStartY, Math.toRadians(intakeStartRotation));
    private final Pose intakeEnd = new Pose(intakeEndX, intakeEndY, Math.toRadians(intakeEndRotation));
    private final Pose scorePose2 = new Pose(scorePose2X, scorePose2Y, Math.toRadians(scorePose2Rotation));
    private final Pose parkPose = new Pose(parkPoseX, parkPoseY, Math.toRadians(parkPoseRotation));

    private PathChain driveToShoot1, driveToIntake, driveToShoot2, driveToPark;

    // Building the paths based on the new visualizer points
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

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, scorePose2))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, parkPose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    // Logic for the shooter and reloading the next ball
    public Command combinedShootLogic() {
        return sequential(
                // Wait until the flywheel is at target speed
                waitUntil(() -> Math.abs(shooter.getVelocity() - ShooterConfig.SHOOTER_VEL_LONG) < ShooterConfig.TPS_TOL),
                
                // Start feeding the ball into the shooter using side power long for stability
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER_LONG)),

                // Wait for the ball to clear the handoff sensor
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),
                
                // Start pulling in the next ball immediately
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // 4s settled wait for long distance shots
                waitMs(5000),

                // Idle at half speed
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(600);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    // The full autonomous sequence
    public Command autoRoutine() {
        return sequential(
                // Score the preload
                parallel(
                        follow(follower, driveToShoot1, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_LONG))
                ),
                combinedShootLogic(),

                // Drive through the intake zone and start grabbing balls immediately
                // We slow down to 50% power for accuracy while picking up, then speed back up
                instant(() -> follower.setMaxPower(0.5)),
                parallel(
                        follow(follower, driveToIntake, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                instant(() -> follower.setMaxPower(1.0)),

                // Score the second cycle (3 balls)
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_LONG))
                ),
                combinedShootLogic(),

                // Final move to the park position
                follow(follower, driveToPark, true),

                // Shutdown
                instant(() -> shooter.setVelocity(0))
        );
    }

    @Override
    public void runOpMode() {
        // Initialize all hardware components
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareConfig.SHOOTER_MOTOR);
        feed = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        // Hardware behavior setup
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));
        
        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("SmallTriRed Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Kick off the auto command
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Telemetry updates for the drivers
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
