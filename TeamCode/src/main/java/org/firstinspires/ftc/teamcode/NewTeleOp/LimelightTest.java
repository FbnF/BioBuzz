package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.VisionService;

@Configurable
@TeleOp(name = "LimelightTest", group = "Test")
public class LimelightTest extends LinearOpMode {

    private final VisionService vision = VisionService.getInstance();

    // Configurable IDs for testing in Panels or FTC Dashboard
    public static int testId1 = 39;
    public static int testId2 = 43;
    public static int testId3 = 35;
    public static int testId4 = 31;

    @Override
    public void runOpMode() {
        vision.init(hardwareMap);
        vision.reset();
        vision.setVisionEnabled(true);

        telemetry.addLine("LimelightTest Initialized.");
        telemetry.addData("Testing Tag IDs", "%d, %d, %d, %d", testId1, testId2, testId3, testId4);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            vision.setGoalTagIds(testId1, testId2, testId3, testId4);
            vision.update();

            telemetry.addLine("=== Limelight AprilTag Test ===");
            telemetry.addData("Target IDs Configured", "%d, %d, %d, %d", testId1, testId2, testId3, testId4);
            telemetry.addData("Target Visible (Matched)", vision.isTargetVisible());
            telemetry.addData("Currently Seeing Goal Tag", vision.isCurrentlySeeing());
            telemetry.addData("Distance (in)", "%.2f", vision.getDistance());
            telemetry.addData("Tx (deg)", "%.2f", vision.getTx());
            telemetry.addData("Ty (deg)", "%.2f", vision.getTy());
            telemetry.update();
        }
    }
}
