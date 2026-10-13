package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.NewTeleOp.Services.RobotHardware;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.VisionService;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.PDController;

@TeleOp(name = "SystemsTest", group = "Testing")
public class SystemsTest extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private VisionService vision = VisionService.getInstance();
    private Follower follower;
    private PDController alignController;

    private boolean intakeOn = false;
    private boolean sideOn = false;
    private boolean feedOn = false;

    private double targetTPS = 1175.0; // Starting default TPS

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousBack = false;
    private boolean previousRightBumper = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;

    private double previousRightTrigger = 0;
    private double previousLeftTrigger = 0;

    @Override
    public void runOpMode() {
        // Enable bulk caching for fast I2C reads
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robot.init(hardwareMap);
        vision.init(hardwareMap);
        vision.reset();
        vision.setVisionEnabled(true);
        // Blue alliance 8 IDs only
        vision.setGoalTagIds(44, 43, 42, 38, 39, 40); // 45 and 41 filtered out
        vision.setIsBlue(true);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive(true);

        alignController = new PDController(ShooterConfig.ALIGN_KP, ShooterConfig.ALIGN_KD);

        robot.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launchMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(500, 3, 0, 4)
        );

        telemetry.addLine("SystemsTest Ready (Pedro + Blue Limelight Auto-Align).");
        telemetry.addLine("A = Flywheel OFF");
        telemetry.addLine("B = Intake + Transfer (Side Servo) ON/OFF toggle");
        telemetry.addLine("X = Feed Servo ON/OFF toggle");
        telemetry.addLine("Y = Everything ON");
        telemetry.addLine("Back = Everything OFF");
        telemetry.addLine("Left Bumper (Hold) = Auto-Align to Blue Goal");
        telemetry.addLine("RB/RT/LT/D-Pad = Flywheel TPS tuning");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            vision.update();

            // Drivetrain & Auto-Align
            double axial = -gamepad1.left_stick_y * 0.95;
            double lateral = -gamepad1.left_stick_x * 0.95;
            double turn = -gamepad1.right_stick_x * 0.95;

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

            double txError = (targetTx - tx) - ShooterConfig.BLUE_ALIGN_OFFSET;

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

            // A Button = Flywheel OFF only
            if (gamepad1.a && !previousA) {
                targetTPS = 0;
                robot.launchMotor.setVelocity(0);
            }

            // B Button = Intake + Transfer (Side Servo) ON/OFF toggle together
            if (gamepad1.b && !previousB) {
                intakeOn = !intakeOn;
                sideOn = intakeOn;
                robot.intakeMotor.setPower(intakeOn ? ShooterConfig.INTAKE_POWER : 0);
                robot.sideServo.setPower(sideOn ? ShooterConfig.SIDE_DEFAULT : 0);
            }

            // X Button = Feed Servo ON/OFF toggle
            if (gamepad1.x && !previousX) {
                feedOn = !feedOn;
                robot.feedServo.setPower(feedOn ? ShooterConfig.FEED_DEFAULT : 0);
            }

            // Y Button = Everything ON
            if (gamepad1.y && !previousY) {
                intakeOn = true;
                sideOn = true;
                feedOn = true;
                targetTPS = ShooterConfig.SHOOTER_VEL_SHORT;

                robot.intakeMotor.setPower(ShooterConfig.INTAKE_POWER);
                robot.sideServo.setPower(ShooterConfig.SIDE_DEFAULT);
                robot.feedServo.setPower(ShooterConfig.FEED_DEFAULT);
                robot.launchMotor.setVelocity(targetTPS);
            }

            // Back Button = Everything OFF
            if (gamepad1.back && !previousBack) {
                intakeOn = false;
                sideOn = false;
                feedOn = false;
                targetTPS = 0;

                robot.intakeMotor.setPower(0);
                robot.sideServo.setPower(0);
                robot.feedServo.setPower(0);
                robot.launchMotor.setVelocity(0);
            }

            // Sync blockServo with cam-jet (feedOn)
            if (feedOn) {
                robot.blockServo.setPosition(0.38); // go
            } else {
                robot.blockServo.setPosition(0.15); // block
            }

            // Flywheel TPS tuning controls
            if (gamepad1.right_bumper && !previousRightBumper) {
                targetTPS += 5;
            }
            if (gamepad1.right_trigger > 0.5 && previousRightTrigger <= 0.5) {
                targetTPS += 10;
            }
            if (gamepad1.left_trigger > 0.5 && previousLeftTrigger <= 0.5) {
                targetTPS -= 10;
            }
            if (gamepad1.dpad_up && !previousDpadUp) {
                targetTPS += 50;
            }
            if (gamepad1.dpad_down && !previousDpadDown) {
                targetTPS -= 50;
            }
            if (gamepad1.dpad_left && !previousDpadLeft) {
                targetTPS = 1175.0;
            }

            targetTPS = Math.max(0, targetTPS);
            if (targetTPS > 0) {
                robot.launchMotor.setVelocity(targetTPS);
            } else {
                robot.launchMotor.setPower(0);
            }

            // Edge detection state updates
            previousA = gamepad1.a;
            previousB = gamepad1.b;
            previousX = gamepad1.x;
            previousY = gamepad1.y;
            previousBack = gamepad1.back;
            previousRightBumper = gamepad1.right_bumper;
            previousRightTrigger = gamepad1.right_trigger;
            previousLeftTrigger = gamepad1.left_trigger;
            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;
            previousDpadLeft = gamepad1.dpad_left;

            // Telemetry
            telemetry.addLine("---- Systems Test (Pedro + Blue Limelight) ----");
            telemetry.addData("Vision Seeing", vision.isCurrentlySeeing());
            telemetry.addData("Vision Dist (in)", "%.2f", vision.getDistance());
            telemetry.addData("Vision Tx", "%.2f", vision.getTx());
            telemetry.addData("Auto-Aligning (LB)", gamepad1.left_bumper && vision.isCurrentlySeeing());
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer (Side)", sideOn ? "ON" : "OFF");
            telemetry.addData("Feed", feedOn ? "ON" : "OFF");
            telemetry.addData("Flywheel Target TPS", "%.0f", targetTPS);
            telemetry.addData("Flywheel Actual TPS", "%.0f", robot.launchMotor.getVelocity());
            telemetry.update();
        }

        // Safety shutdown
        robot.launchMotor.setVelocity(0);
        robot.launchMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        robot.sideServo.setPower(0);
        robot.feedServo.setPower(0);
    }
}
