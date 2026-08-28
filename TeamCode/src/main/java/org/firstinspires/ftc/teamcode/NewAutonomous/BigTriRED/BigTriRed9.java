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
@Autonomous(name = "BigTriRED9", group = "Autonomous")
public class BigTriRed9 extends LinearOpMode {

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

    public static double intakeStart1X = 95.014;
    public static double intakeStart1Y = 81.243;
    public static double intakeStart1Rotation = 0;

    public static double intakeEnd1X = 125.461;
    public static double intakeEnd1Y = 80.865;
    public static double intakeEnd1Rotation = 0;

    public static double scorePose2X = 82.728;
    public static double scorePose2Y = 101.335;
    public static double scorePose2Rotation = 34;

    public static double intakeStart2X = 95.104;
    public static double intakeStart2Y = 59;
    public static double intakeStart2Rotation = 0;

    public static double intakeEnd2X = 129;
    public static double intakeEnd2Y = 59;
    public static double intakeEnd2Rotation = 0;

    public static double scorePose3X = 82.728;
    public static double scorePose3Y = 101.335;
    public static double scorePose3Rotation = 34;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose scorePose1 = new Pose(scorePose1X, scorePose1Y, Math.toRadians(scorePose1Rotation));
    private final Pose intakeStart1 = new Pose(intakeStart1X, intakeStart1Y, Math.toRadians(intakeStart1Rotation));
    private final Pose intakeEnd1 = new Pose(intakeEnd1X, intakeEnd1Y, Math.toRadians(intakeEnd1Rotation));
    private final Pose scorePose2 = new Pose(scorePose2X, scorePose2Y, Math.toRadians(scorePose2Rotation));
    private final Pose intakeStart2 = new Pose(intakeStart2X, intakeStart2Y, Math.toRadians(intakeStart2Rotation));
    private final Pose intakeEnd2 = new Pose(intakeEnd2X, intakeEnd2Y, Math.toRadians(intakeEnd2Rotation));
    private final Pose scorePose3 = new Pose(scorePose3X, scorePose3Y, Math.toRadians(scorePose3Rotation));



    //-------------------- Defined Paths --------------------
    private PathChain driveToShoot1, driveToIntake1, driveToShoot2, driveToIntake2, driveThroughLine2, driveToShoot3;

    public void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intakeStart1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakeStart1.getHeading())
                .addPath(new BezierLine(intakeStart1, intakeEnd1))
                .setConstantHeadingInterpolation(intakeStart1.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd1, scorePose2))
                .setLinearHeadingInterpolation(intakeEnd1.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();

        // Split paths for Rack 2 to prevent right-bias corner cutting
        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, intakeStart2))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), intakeStart2.getHeading())
                .setGlobalDeceleration()
                .build();

        driveThroughLine2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart2, intakeEnd2))
                .setConstantHeadingInterpolation(intakeStart2.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd2, scorePose3))
                .setLinearHeadingInterpolation(intakeEnd2.getHeading(), scorePose3.getHeading())
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

                // Idle at half speed
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(600); 
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }



    //-------------------- Auto Routine --------------------
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

                // Shot 2 + Early Start
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Pickup 2 - Forces snap to start position to fix overshooting right
                follow(follower, driveToIntake2, true),
                
                instant(() -> follower.setMaxPower(0.6)),
                parallel(
                        follow(follower, driveThroughLine2, true),
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
