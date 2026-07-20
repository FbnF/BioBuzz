package org.firstinspires.ftc.teamcode.NewAutonomous.SmallTriangle;

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

@Autonomous(name = "SmallBlueSimple", group = "Autonomous")
public class SmallBlueSimple extends LinearOpMode {

    private Follower follower;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo feed;
    private CRServo sideServo;
    private DistanceSensor rangeSensor;

    // Poses derived from the visualizer
    private final Pose startPose = new Pose(55.377, 7.861, Math.toRadians(90));
    private final Pose scorePose = new Pose(58.446, 16.959, Math.toRadians(110));
    private final Pose parkPose = new Pose(35.726, 16.464, Math.toRadians(90));

    private PathChain driveToShoot, driveToPark;

    // Building the paths using the visualizer points
    public void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration() // Keep it smooth when coming out of the first move
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration() // Smooth landing into the park
                .build();
    }

    //Shooter and feeder combined into one code action
    public Command combinedShootLogic() {
        return sequential(
                // Spin up the shooter with the long distance velocity
                instant(() -> shooter.setVelocity(ShooterConfig.SHOOTER_VEL_LONG)),
                
                // Give it a second to get up to speed
                waitMs((long)(ShooterConfig.START_WAIT_TIME * 1000)),
                
                // Start feeding the ball into the shooter
                instant(() -> feed.setPower(ShooterConfig.SIDE_POWER)),

                // Wait for the range sensor to tell us the ball is gone
                // Then kick on the intake and side servo to grab the next one
                waitUntil(() -> rangeSensor.getDistance(DistanceUnit.MM) > ShooterConfig.HANDOFF_DISTANCE_MM),
                parallel(
                        instant(() -> sideServo.setPower(1.0)),
                        instant(() -> intake.setPower(ShooterConfig.INTAKE_POWER))
                ),

                // Keep everything running for the full shot time
                waitMs((long)((ShooterConfig.WAIT_TIME - ShooterConfig.START_WAIT_TIME) * 1000)),

                // Shut it all down
                instant(() -> {
                    feed.setPower(0);
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    sideServo.setPower(0);
                })
        );
    }

    // The main auto routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, driveToShoot, true), // Hold ground while shooting
                combinedShootLogic(),           
                follow(follower, driveToPark, true)   // Hold ground in the park
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

        telemetry.addLine("SmallBlueSimple Ready.");
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
