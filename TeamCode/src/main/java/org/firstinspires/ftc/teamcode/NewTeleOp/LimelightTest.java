package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.VisionService;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.PDController;

@Configurable
@TeleOp(name = "LimelightTest", group = "Test")
public class LimelightTest extends LinearOpMode {

    private final VisionService vision = VisionService.getInstance();
    private Follower follower;
    private PDController alignController;

    private boolean isBlueAlliance = true; // true = Blue [8 IDs], false = Red [8 IDs]
    private boolean prevAllianceButton = false;
    private final double speedFactor = 0.8;

    @Override
    public void runOpMode() {
        // Enable bulk caching for fast I2C reads and loop time optimization
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive(true);

        vision.init(hardwareMap);
        vision.reset();
        vision.setVisionEnabled(true);
        // Default to Blue 8 IDs
        vision.setGoalTagIds(45, 44, 43, 42, 38, 39, 40, 41);
        vision.setIsBlue(true);

        alignController = new PDController(ShooterConfig.ALIGN_KP, ShooterConfig.ALIGN_KD);

        telemetry.addLine("LimelightTest Initialized (Bulk Caching Enabled).");
        telemetry.addLine("Press Dpad_Down to toggle Alliance (Blue [8 IDs] <-> Red [8 IDs])");
        telemetry.addLine("Hold Left Bumper to Auto-Align");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            vision.update();

            // Alliance toggle button (Dpad_Down with edge detection)
            boolean allianceBtn = gamepad1.dpad_down;
            if (allianceBtn && !prevAllianceButton) {
                isBlueAlliance = !isBlueAlliance;
                if (isBlueAlliance) {
                    vision.setGoalTagIds(45, 44, 43, 42, 38, 39, 40, 41);
                    vision.setIsBlue(true);
                } else {
                    vision.setGoalTagIds(33, 32, 31, 30, 34, 35, 36, 37);
                    vision.setIsBlue(false);
                }
            }
            prevAllianceButton = allianceBtn;

            // Drivetrain controls
            double axial = -gamepad1.right_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x * speedFactor;
            double turn = -gamepad1.right_stick_x * speedFactor;

            // Auto-align logic (Left Bumper)
            double[] win = vision.getTxWindow();
            double txMin = win[0];
            double txMax = win[1];
            double tx = vision.getTx();
            double targetTx;

            if (tx < txMin || tx > txMax) {
                targetTx = (txMin + txMax) / 2.0;
            } else {
                targetTx = tx;
            }

            double currentOffset = isBlueAlliance ? ShooterConfig.BLUE_ALIGN_OFFSET : ShooterConfig.RED_ALIGN_OFFSET;
            double txError = (targetTx - tx) - currentOffset;

            if (ShooterConfig.AUTO_ALIGN_ENABLED && gamepad1.left_bumper
                    && vision.isVisionEnabled() && vision.isCurrentlySeeing()) {
                if (Math.abs(txError) <= ShooterConfig.ALIGN_ERR_DEADBAND_DEG) {
                    turn = 0;
                } else {
                    turn = alignController.update(txError);
                    if (Math.abs(turn) > 0 && Math.abs(turn) < ShooterConfig.ALIGN_MIN_TURN) {
                        turn = Math.copySign(ShooterConfig.ALIGN_MIN_TURN, turn);
                    }
                }
            } else {
                alignController.reset();
            }

            follower.setTeleOpDrive(axial, lateral, turn, true);

            // Telemetry
            telemetry.addLine("=== Limelight Test & Auto-Align ===");
            telemetry.addData("Active Alliance", isBlueAlliance ? "BLUE [45,44,43,42,38,39,40,41]" : "RED [33,32,31,30,34,35,36,37]");
            telemetry.addData("Target Visible", vision.isTargetVisible());
            telemetry.addData("Currently Seeing", vision.isCurrentlySeeing());
            telemetry.addData("Distance (in)", "%.2f", vision.getDistance());
            telemetry.addData("Tx (deg)", "%.2f", vision.getTx());
            telemetry.addData("Ty (deg)", "%.2f", vision.getTy());
            telemetry.addData("Auto-Aligning (Left Bumper)", gamepad1.left_bumper && vision.isCurrentlySeeing());
            telemetry.update();
        }
    }
}
