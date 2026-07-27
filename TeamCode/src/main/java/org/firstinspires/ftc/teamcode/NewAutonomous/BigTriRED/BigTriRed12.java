package org.firstinspires.ftc.teamcode.NewAutonomous.BigTriRED;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BigTriRED12", group = "Autonomous")
public class BigTriRed12 extends LinearOpMode {

    //-------------------- Hardware & Follower Constants --------------------
    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;



    //-------------------- POSES --------------------
    private final Pose startPose = new Pose(108.582, 132.904, Math.toRadians(270));
    private final Pose scorePose1 = new Pose(92.510, 91.727, Math.toRadians(45));
    private final Pose intakeStart1 = new Pose(95.014, 81.243, Math.toRadians(0));
    private final Pose intakeEnd1 = new Pose(123, 80.865, Math.toRadians(0));
    private final Pose clearPose = new Pose(127.00171789185484, 75, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(96.78083109919571, 87.97184986595173, Math.toRadians(52));
    private final Pose intakeStart2 = new Pose(95.104, 59, Math.toRadians(7));
    private final Pose intakeEnd2 = new Pose(128, 59, Math.toRadians(7));
    private final Pose scorePose3 = new Pose(96.78083109919571, 87.97184986595173, Math.toRadians(55));
    private final Pose clearAndCollect = new Pose(129.12416107382552,59.931543624161066, Math.toRadians(28));
    private final Pose clearAndCollect2 = new Pose(130,63, Math.toRadians(28));

    private final Pose scorePose4 = new Pose(82.728, 101.335, Math.toRadians(35));



    //-------------------- Defined Paths --------------------
    private PathChain driveToShoot1, driveToIntake1, driveToClear, driveToShoot2, driveToIntake2, driveThroughLine2, driveToShoot3, driveToIntake3,driveToShoot4;

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
        driveToClear = follower.pathBuilder()
                .addPath(new BezierCurve(intakeEnd1, new Pose(117.16562898525812,76.21853095574171) , clearPose))
                .setLinearHeadingInterpolation(intakeEnd1.getHeading(), clearPose.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(clearPose, scorePose2))
                .setLinearHeadingInterpolation(clearPose.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();

        // Split paths for line 2 to prevent right-bias corner cutting
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
                .addPath(new BezierLine(intakeEnd2, scorePose2))
                .setLinearHeadingInterpolation(intakeEnd2.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();
        driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2,clearAndCollect))
                .setLinearHeadingInterpolation(scorePose2.getHeading(),clearAndCollect.getHeading())
                .addPath(new BezierLine(clearAndCollect,clearAndCollect2))
                .setLinearHeadingInterpolation(clearAndCollect.getHeading(),clearAndCollect2.getHeading())
                .setGlobalDeceleration()
                .build();
        driveToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(clearAndCollect, scorePose4))
                .setLinearHeadingInterpolation(clearAndCollect.getHeading(), scorePose4.getHeading())
                .setGlobalDeceleration()
                .build();
    }



    //-------------------- Shooter & Reload Logic --------------------
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

                // 2.8s settled wait
                waitMs(2500),

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
                instant(() -> follower.setMaxPower(1.0)),
                parallel(
                        follow(follower, driveToShoot1, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                //pick up first spike line
                parallel(
                        follow(follower, driveToIntake1, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                //clear rack
                follow(follower, driveToClear, true),
                waitMs(500),

                // Shot 2 + Early Start
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Pickup 2 - Forces snap to start position to fix overshooting right
                follow(follower, driveToIntake2, true),
                parallel(
                        follow(follower, driveThroughLine2, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),

                // Shot 3 + Early Start
                parallel(
                        follow(follower, driveToShoot3, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),
                instant(() -> {
                    intake.setPower(1.0);
                    sideServo.setPower(1.0);
                }),

                //clears and collects 3 artifacts
                follow(follower, driveToIntake3, true),
                waitMs(1200),

                parallel(
                        follow(follower, driveToShoot4, true),
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
