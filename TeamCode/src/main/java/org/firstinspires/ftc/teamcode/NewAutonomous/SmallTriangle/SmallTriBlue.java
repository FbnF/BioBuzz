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
@Autonomous(name = "SmallTriBlue", group = "Autonomous")
public class SmallTriBlue extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Latest Poses from the Blue Side Visualizer
    public static double startPoseX = 55.377;
    public static double startPoseY = 7.861;
    public static double startPoseRotation = 90;

    public static double scorePose1X = 58.446;
    public static double scorePose1Y = 16.959;
    public static double scorePose1Rotation = 111;

    public static double intakeStartX = 46.778;
    public static double intakeStartY = 35.749;
    public static double intakeStartRotation = 180;

    public static double intakeEndX = 14.798;
    public static double intakeEndY = 36.435;
    public static double intakeEndRotation = 180;

    public static double scorePose2X = 58.446;
    public static double scorePose2Y = 16.959;
    public static double scorePose2Rotation = 112;

    public static double parkPoseX = 36.599;
    public static double parkPoseY = 16.115;
    public static double parkPoseRotation = 90;

    // Latest Poses from the Blue Side Visualizer
    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose scorePose1 = new Pose(scorePose1X, scorePose1Y, Math.toRadians(scorePose1Rotation));
    private final Pose intakeStart = new Pose(intakeStartX, intakeStartY, Math.toRadians(intakeStartRotation));
    private final Pose intakeEnd = new Pose(intakeEndX, intakeEndY, Math.toRadians(intakeEndRotation));
    private final Pose scorePose2 = new Pose(scorePose2X, scorePose2Y, Math.toRadians(scorePose2Rotation));
    private final Pose parkPose = new Pose(parkPoseX, parkPoseY, Math.toRadians(parkPoseRotation));

    private PathChain driveToShoot1, driveToIntake, driveToShoot2, driveToPark;

    // Setting up the path chains with global deceleration for smoothness
    public void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .setGlobalDeceleration()
                .build();

        // Driving towards the wall to grab the floor balls
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

    // Bundled logic for the shooter and the auto-reload sequence
    public Command combinedShootLogic() {
        return sequential(
                // Wait until the flywheel is at target speed
                waitUntil(() -> Math.abs(shooter.getVelocity() - ShooterConfig.SHOOTER_VEL_LONG) < ShooterConfig.TPS_TOL),
                
                // Start feeding the ball into the shooter using side power long for stability
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER_LONG)),

                // Wait for the handoff to clear
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),
                
                // Immediately start the intake reload
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

    // The complete autonomous command structure
    public Command autoRoutine() {
        return sequential(
                // Initial shot
                parallel(
                        follow(follower, driveToShoot1, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_LONG))
                ),
                combinedShootLogic(),

                // Slow down to 50% power for a clean pickup towards the wall
                instant(() -> follower.setMaxPower(0.5)),
                parallel(
                        follow(follower, driveToIntake, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                // Back to full power once we have the goods
                instant(() -> follower.setMaxPower(1.0)),

                // Return and score
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_LONG))
                ),
                combinedShootLogic(),

                // Park it
                follow(follower, driveToPark, true),

                // Shutdown
                instant(() -> shooter.setVelocity(0))
        );
    }

    @Override
    public void runOpMode() {
        // Initialize hardware using our configs
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareConfig.SHOOTER_MOTOR);
        feed = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        // Subsystem safety and encoder setup
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));
        
        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("SmallTriBlue Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Schedule the routine into the Ivy loop
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Status telemetry for the driver station
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
