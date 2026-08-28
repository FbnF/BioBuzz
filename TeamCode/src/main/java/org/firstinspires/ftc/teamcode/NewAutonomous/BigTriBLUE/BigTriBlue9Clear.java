package org.firstinspires.ftc.teamcode.NewAutonomous.BigTriBLUE;

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
@Autonomous(name = "BigTriBLUE9Clear", group = "Autonomous")
public class BigTriBlue9Clear extends LinearOpMode {

    //-------------------- Hardware & Follower Constants --------------------
    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    private double shooter_Vel = 1160.0;



    //-------------------- POSES --------------------
    public static double startPoseX = 33.814;
    public static double startPoseY = 133.428;
    public static double startPoseRotation = 270;

    public static double scorePose1X = 49.362;
    public static double scorePose1Y = 91.028;
    public static double scorePose1Rotation = 130;

    public static double intakeStart1X = 46.806;
    public static double intakeStart1Y = 85.046;
    public static double intakeStart1Rotation = 180;

    public static double intakeEnd1X = 16.983;
    public static double intakeEnd1Y = 84.431;
    public static double intakeEnd1Rotation = 180;
    
    // Clear maneuver poses
    public static double clearStartX = 22.079;
    public static double clearStartY = 76;
    public static double clearStartRotation = 180;

    public static double clearEndX = 13;
    public static double clearEndY = 76;
    public static double clearEndRotation = 180;
    

    public static double scorePose2X = 57.572;
    public static double scorePose2Y = 98.889;
    public static double scorePose2Rotation = 145;

    public static double intakeStart2X = 47.273;
    public static double intakeStart2Y = 64.785;
    public static double intakeStart2Rotation = 180;

    public static double intakeEnd2X = 19;
    public static double intakeEnd2Y = 63.785;
    public static double intakeEnd2Rotation = 180;

    public static double scorePose3X = 57.398;
    public static double scorePose3Y = 98.889;
    public static double scorePose3Rotation = 145;

    private final Pose startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseRotation));
    private final Pose scorePose1 = new Pose(scorePose1X, scorePose1Y, Math.toRadians(scorePose1Rotation));
    private final Pose intakeStart1 = new Pose(intakeStart1X, intakeStart1Y, Math.toRadians(intakeStart1Rotation));
    private final Pose intakeEnd1 = new Pose(intakeEnd1X, intakeEnd1Y, Math.toRadians(intakeEnd1Rotation));
    
    // Clear maneuver poses
    private final Pose clearStart = new Pose(clearStartX, clearStartY, Math.toRadians(clearStartRotation));
    private final Pose clearEnd = new Pose(clearEndX, clearEndY, Math.toRadians(clearEndRotation));
    
    private final Pose scorePose2 = new Pose(scorePose2X, scorePose2Y, Math.toRadians(scorePose2Rotation));
    private final Pose intakeStart2 = new Pose(intakeStart2X, intakeStart2Y, Math.toRadians(intakeStart2Rotation));
    private final Pose intakeEnd2 = new Pose(intakeEnd2X, intakeEnd2Y, Math.toRadians(intakeEnd2Rotation));
    private final Pose scorePose3 = new Pose(scorePose3X, scorePose3Y, Math.toRadians(scorePose3Rotation));




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
                // Wait until the flywheel is at target speed
                waitUntil(() -> Math.abs(shooter.getVelocity() - shooter_Vel) < ShooterConfig.TPS_TOL),

                // Engagement using config SIDE_POWER (-1.0)
                instant(() -> feed.setPower(-0.9)),

                // Wait for fire
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),

                // Start reload
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // 4.2s settled wait
                waitMs(3500),

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
                
                // Clear maneuver (with 1s timeout to prevent getting stuck at the gate)
                follow(follower, driveToClear, true).raceWith(waitMs(2000)),
                waitMs(250),
                
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
