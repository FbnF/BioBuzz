package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.RobotHardware;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

/**
 * simple TeleOp to test the flywheel at specific speeds.
 * A: 720 TPS
 * B: Short Velocity
 * X: Long Velocity
 * Y: Stop
 */
@TeleOp(name = "ShooterTest", group = "TeleOp")
public class ShooterTest extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private double targetTPS = 0;

    @Override
    public void runOpMode() {
        // Init hardware using established service
        robot.init(hardwareMap);

        telemetry.addLine("ShooterTest Ready.");
        telemetry.addData("A", "720 TPS");
        telemetry.addData("B", "Short (%.0f)", ShooterConfig.SHOOTER_VEL_SHORT);
        telemetry.addData("X", "Long (%.0f)", ShooterConfig.SHOOTER_VEL_LONG);
        telemetry.addData("Y", "Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Button mapping
            if (gamepad1.a) targetTPS = 720;
            if (gamepad1.b) targetTPS = ShooterConfig.SHOOTER_VEL_SHORT;
            if (gamepad1.x) targetTPS = ShooterConfig.SHOOTER_VEL_LONG;
            if (gamepad1.y) targetTPS = 0;

            // Apply velocity
            robot.launchMotor.setVelocity(targetTPS);

            // Feedback
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", robot.launchMotor.getVelocity());
            telemetry.update();
        }

        // Safety shutdown
        robot.launchMotor.setPower(0);
    }
}
