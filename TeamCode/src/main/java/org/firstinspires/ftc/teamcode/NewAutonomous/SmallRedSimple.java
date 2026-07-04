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

@Autonomous(name = "SmallRedSimple", group = "Autonomous")
public class SmallRedSimple extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Poses from Visualizer (Starting facing goal side @ 90 degrees)
    private final Pose startPose = new Pose(86.298, 8.385, Math.toRadians(90));
    private final Pose scorePose = new Pose(82.378, 18.531, Math.toRadians(65));
    private final Pose parkPose = new Pose(105.253, 16.638, Math.toRadians(90));

    private PathChain driveToShoot, driveToPark;

    public void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    //Shooter and feeder combined into one code action
    public Command combinedShootLogic() {
        return sequential(
                //Spin up shooter using config value
                instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_LONG)),
                
                //Wait for motor to reach speed (START_WAIT_TIME)
                waitMs((long)(ShooterConfig.START_WAIT_TIME * 1000)),
                
                //Start feeding the ball using SIDE_POWER config
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER)),

                // Wait for ball to clear (HANDOFF_DISTANCE_MM)
                // When ball leaves, kick on intake and side servo to reload
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // Wait for the remainder of the shot duration (WAIT_TIME)
                waitMs((long)((ShooterConfig.WAIT_TIME - ShooterConfig.START_WAIT_TIME) * 1000)),

                // Stop all motors
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, driveToShoot, true),
                combinedShootLogic(),
                follow(follower, driveToPark, true)
        );
    }

    @Override
    public void runOpMode() {
        // Init Hardware using HardwareConfig names
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareConfig.SHOOTER_MOTOR);
        feed = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);

        // Motor Setup
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));
        
        Scheduler.reset();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("SmallRedSimple Ready.");
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
            telemetry.addData("Ball Distance (mm)", rangeSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
