package org.firstinspires.ftc.teamcode.Teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class CPUTest extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Limelight3A LimeLight;
    private LLStatus status;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        LimeLight = hardwareMap.get(Limelight3A.class, "Limelight");
        LimeLight.setPollRateHz(100);
        LimeLight.start();


        LimeLight.pipelineSwitch(1);
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        LLStatus status = LimeLight.getStatus();
        readLimelightAI();
        double cpuUsage = status.getCpu();
        double cpuTemp = status.getTemp();
        double ramUsage = status.getRam();
        int fps = (int) status.getFps();

        telemetry.addData("Limelight CPU", cpuUsage + "%");
        telemetry.addData("Limelight Temp", cpuTemp + "°C");
        telemetryM.update();
        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode onb
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }
        telemetryM.addData("position", follower.getPose());
        telemetryM.addData("velocity", follower.getVelocity());
        telemetryM.addData("automatedDrive", automatedDrive);
        telemetry.update();
    }
    public void readLimelightAI() {
        LLResult result = LimeLight.getLatestResult();

        if (result != null && result.isValid()) {
            double mainTargetX = result.getTx();
            double mainTargetY = result.getTy();
            double mainTargetArea = result.getTa();

            telemetry.addData("Main Target", "X: " + mainTargetX + ", Y: " + mainTargetY);
        }
    }
}