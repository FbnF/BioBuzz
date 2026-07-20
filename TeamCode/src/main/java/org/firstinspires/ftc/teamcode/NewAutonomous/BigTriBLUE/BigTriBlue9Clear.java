package org.firstinspires.ftc.teamcode.NewAutonomous.BigTriBLUE;

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

@Autonomous(name = "BigTriBLUE9Clear", group = "Autonomous")
public class BigTriBlue9Clear extends LinearOpMode {

    //-------------------- Hardware & Follower Constants --------------------
    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;



    //-------------------- POSES --------------------
    private final Pose startPose = new Pose(33.814, 133.428, Math.toRadians(270));
    private final Pose scorePose1 = new Pose(49.362, 91.028, Math.toRadians(135));
    private final Pose intakeStart1 = new Pose(46.806, 85.046, Math.toRadians(180));
    private final Pose intakeEnd1 = new Pose(16.983, 84.431, Math.toRadians(180));
    
    // Clear maneuver poses
    private final Pose clearStart = new Pose(29.416, 76.200, Math.toRadians(270));
    private final Pose clearEnd = new Pose(13.797, 76.364, Math.toRadians(270));
    
    private final Pose scorePose2 = new Pose(58.271, 100.461, Math.toRadians(145));
    private final Pose intakeStart2 = new Pose(47.273, 64.785, Math.toRadians(180));
    private final Pose intakeEnd2 = new Pose(15.293, 64.724, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(58.271, 100.461, Math.toRadians(145));



    //-------------------- Defined Paths --------------------
    private PathChain driveToShoot1, driveToIntake1, driveToClear, driveToShoot2, driveToIntake2, driveToShoot3;

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
                .addPath(new BezierLine(intakeEnd1, clearStart))
                .setLinearHeadingInterpolation(intakeEnd1.getHeading(), clearStart.getHeading())
                .addPath(new BezierLine(clearStart, clearEnd))
                .setConstantHeadingInterpolation(clearStart.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(clearEnd, scorePose2))
                .setLinearHeadingInterpolation(clearEnd.getHeading(), scorePose2.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, intakeStart2))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), intakeStart2.getHeading())
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
                waitMs(3000),

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

                // Cycle 1
                instant(() -> follower.setMaxPower(0.5)),
                parallel(
                        follow(follower, driveToIntake1, true),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER)),
                        instant(() -> sideServo.setPower(1.0))
                ),
                
                // Clear maneuver
                follow(follower, driveToClear, true),
                waitMs(150), // waitSeconds(0.15) from RR
                
                instant(() -> follower.setMaxPower(1.0)),

                // Shot 2 + Early Start
                parallel(
                        follow(follower, driveToShoot2, true),
                        instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_SHORT))
                ),
                combinedShootLogic(),

                // Cycle 2
                instant(() -> follower.setMaxPower(0.5)),
                parallel(
                        follow(follower, driveToIntake2, true),
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

        telemetry.addLine("BigTriBLUE9Clear Ready.");
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
