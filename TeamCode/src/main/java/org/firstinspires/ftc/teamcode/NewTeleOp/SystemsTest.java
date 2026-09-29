package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.NewTeleOp.Services.RobotHardware;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

@TeleOp(name = "SystemsTest", group = "Testing")
public class SystemsTest extends LinearOpMode {

    // Controls:
    // A = Intake On/Off
    // B = Transfer On/Off
    // X = Launch Servo On/Off
    // Y = Everything On
    // LB = Everything Off
    // D-Pad Up = 720 TPS
    // D-Pad Right = Short Speed
    // D-Pad Down = Long Speed
    // D-Pad Left = Flywheel Off

    private RobotHardware robot = RobotHardware.getInstance();

    private boolean intakeOn = false;
    private boolean sideOn = false;
    private boolean feedOn = false;

    private double targetTPS = 0;

    @Override
    public void runOpMode() {

        // Init hardware
        robot.init(hardwareMap);

        robot.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.launchMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(500, 3, 0, 4)
        );

        telemetry.addLine("SystemsTest Ready.");
        telemetry.addLine("");
        telemetry.addLine("A = Intake On/Off");
        telemetry.addLine("B = Transfer On/Off");
        telemetry.addLine("X = Launch On/Off");
        telemetry.addLine("Y = Everything On");
        telemetry.addLine("LB = Everything Off");
        telemetry.addLine("");
        telemetry.addLine("D-Pad Up = 720 TPS");
        telemetry.addLine("D-Pad Right = Short");
        telemetry.addLine("D-Pad Down = Long");
        telemetry.addLine("D-Pad Left = Flywheel Off");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake
            if (gamepad1.a) {
                intakeOn = !intakeOn;
                robot.intakeMotor.setPower(
                        intakeOn ? ShooterConfig.INTAKE_POWER : 0
                );
                sleep(150);
            }

            // Transfer / Side Servo
            if (gamepad1.b) {
                sideOn = !sideOn;
                robot.sideServo.setPower(
                        sideOn ? ShooterConfig.SIDE_DEFAULT : 0
                );
                sleep(150);
            }

            // Launch / Feed Servo
            if (gamepad1.x) {
                feedOn = !feedOn;
                robot.feedServo.setPower(
                        feedOn ? ShooterConfig.FEED_DEFAULT : 0
                );
                sleep(150);
            }

            // Everything On
            if (gamepad1.y) {
                intakeOn = true;
                sideOn = true;
                feedOn = true;

                robot.intakeMotor.setPower(ShooterConfig.INTAKE_POWER);
                robot.sideServo.setPower(ShooterConfig.SIDE_DEFAULT);
                robot.feedServo.setPower(ShooterConfig.FEED_DEFAULT);

                sleep(150);
            }

            // Everything Off
            if (gamepad1.left_bumper) {
                intakeOn = false;
                sideOn = false;
                feedOn = false;
                targetTPS = 0;

                robot.intakeMotor.setPower(0);
                robot.sideServo.setPower(0);
                robot.feedServo.setPower(0);
                robot.launchMotor.setVelocity(0);

                sleep(150);
            }

            // Flywheel Speeds
            if (gamepad1.dpad_up) {
                targetTPS = 720;
                robot.launchMotor.setVelocity(targetTPS);
                sleep(150);
            }

            if (gamepad1.dpad_right) {
                targetTPS = ShooterConfig.SHOOTER_VEL_SHORT;
                robot.launchMotor.setVelocity(targetTPS);
                sleep(150);
            }

            if (gamepad1.dpad_down) {
                targetTPS = ShooterConfig.SHOOTER_VEL_LONG;
                robot.launchMotor.setVelocity(targetTPS);
                sleep(150);
            }

            if (gamepad1.dpad_left) {
                targetTPS = 0;
                robot.launchMotor.setVelocity(0);
                sleep(150);
            }

            telemetry.addLine("---- Systems Test ----");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", sideOn ? "ON" : "OFF");
            telemetry.addData("Launch", feedOn ? "ON" : "OFF");

            telemetry.addLine("");
            telemetry.addLine("---- Flywheel ----");
            telemetry.addData("Target TPS", "%.0f", targetTPS);
            telemetry.addData("Actual TPS", "%.0f", robot.launchMotor.getVelocity());

            telemetry.update();
        }

        // Safety shutdown
        robot.launchMotor.setVelocity(0);
        robot.intakeMotor.setPower(0);
        robot.sideServo.setPower(0);
        robot.feedServo.setPower(0);
    }
}