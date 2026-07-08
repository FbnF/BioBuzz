package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.utils.PDController;

/**
 * Main Blue TeleOp.
 */
@TeleOp(name = "TeleOpMainBlue", group = "TeleOp")
public class TeleOpMainBlue extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private VisionService vision = VisionService.getInstance();
    private ShooterService shooter = ShooterService.getInstance();
    private LedService leds = LedService.getInstance();
    private LoggingService logger = LoggingService.getInstance();
    
    private Follower follower;
    private PDController alignController;
    private double speedFactor = 0.7;
    
    private boolean autoSpinArmed = true;
    private boolean prevDpadRight = false;
    private boolean prevG2DpadLeft = false;
    private boolean isEjecting = false;
    private long ejectStartNs = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision.init(hardwareMap);
        vision.setGoalTagId(20); 
        shooter.setIsBlue(true);
        
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();

        logger.init(hardwareMap, follower::getPose);
        alignController = new PDController(ShooterConfig.ALIGN_KP, ShooterConfig.ALIGN_KD);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            vision.update();
            leds.update();
            logger.record();

            // ---------------- DRIVE ----------------
            if (gamepad1.a) speedFactor = 1.35;
            if (gamepad1.b) speedFactor = 0.4;
            if (gamepad1.x) speedFactor = 0.7;

            double axial = -gamepad1.left_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x * speedFactor;
            double turn = -gamepad1.right_stick_x * speedFactor;

            // ---------------- VISION TOGGLE ----------------
            if (gamepad2.dpad_left && !prevG2DpadLeft) {
                vision.setVisionEnabled(!vision.isVisionEnabled());
            }
            prevG2DpadLeft = gamepad2.dpad_left;

            // ---------------- AUTO ALIGN ----------------
            double[] win = vision.getTxWindow();
            double targetTx = (win[0] + win[1]) / 2.0;
            double txError = targetTx - vision.getTx();

            if (gamepad1.left_bumper && vision.isTargetVisible()) {
                turn = alignController.update(txError);
            } else {
                alignController.reset();
            }

            follower.setTeleOpDrive(axial, lateral, turn, true);

            // ---------------- SHOOTER ----------------
            if (gamepad2.dpad_right && !prevDpadRight) autoSpinArmed = !autoSpinArmed;
            prevDpadRight = gamepad2.dpad_right;

            double targetTPS = 0;
            if (autoSpinArmed) {
                targetTPS = shooter.calculateVelocity(vision.getDistance(), vision.isTargetVisible());
                robot.launchMotor.setVelocity(targetTPS);
            } else {
                robot.launchMotor.setPower(0);
            }

            // ---------------- FEED & INTAKE ----------------
            boolean motorReady = Math.abs(robot.launchMotor.getVelocity() - targetTPS) < ShooterConfig.TPS_TOL;
            boolean isLoaded = robot.rangeSensor.getDistance(DistanceUnit.MM) < 170;
            boolean angleOk = vision.getTx() >= win[0] && vision.getTx() <= win[1];
            boolean noShotZone = vision.isNoShotZone();

            boolean canShoot = motorReady && autoSpinArmed && vision.isTargetVisible() && angleOk && !noShotZone;

            if (gamepad2.y) {
                if (canShoot) {
                    double dist = vision.getDistance();
                    robot.feedServo.setPower(shooter.getFeedPower(dist));
                    robot.intakeMotor.setPower(1.0);
                    robot.sideServo.setPower(shooter.getSidePower(dist));
                } else {
                    leds.triggerWarning();
                }
            } else if (gamepad2.x) {
                robot.feedServo.setPower(0);
                robot.intakeMotor.setPower(1.0);
                robot.sideServo.setPower(1.0);
            } else if (gamepad2.right_trigger > 0) {
                robot.intakeMotor.setPower(1.0);
                robot.sideServo.setPower(1.0);
            } else if (gamepad2.left_bumper && !isEjecting) {
                isEjecting = true;
                ejectStartNs = System.nanoTime();
                robot.intakeMotor.setPower(-0.7);
            } else if (gamepad2.dpad_down && !isEjecting) {
                // Special Blue ejection mode
                isEjecting = true;
                ejectStartNs = System.nanoTime();
                robot.intakeMotor.setPower(-0.7);
                robot.sideServo.setPower(-0.7);
            } else if (gamepad2.left_trigger > 0) {
                robot.intakeMotor.setPower(0);
                robot.sideServo.setPower(0);
                robot.feedServo.setPower(0);
            } else if (!isEjecting) {
                robot.intakeMotor.setPower(0);
                robot.sideServo.setPower(0);
                if (!motorReady && !isLoaded) robot.feedServo.setPower(0);
            }

            if (isEjecting && (System.nanoTime() - ejectStartNs) / 1e9 >= 0.25) {
                isEjecting = false;
                robot.intakeMotor.setPower(0);
                robot.sideServo.setPower(0);
            }

            leds.setStatus(vision.isTargetVisible(), motorReady, noShotZone);

            // ---------------- TELEMETRY ----------------
            telemetry.addData("ServoSpeed", robot.feedServo.getPower());
            telemetry.addLine("---- Vision Distance ----");
            telemetry.addData("Vision Enabled", vision.isVisionEnabled());
            telemetry.addData("Goal Tag Found", vision.hasGoalTag());
            telemetry.addData("Seen Tag ID", vision.getGoalTagId());
            telemetry.addData("GOAL_TAG_ID", 20);
            telemetry.addData("rangeRawIn", "%.2f", vision.getRawDistance());
            telemetry.addData("rangeFiltIn", "%.2f", vision.getDistance());
            telemetry.addData("Tx", "%.2f", vision.getTx());
            telemetry.addData("Ty", "%.2f", vision.getTy());

            telemetry.addLine("---- Align Window (Tx) ----");
            telemetry.addData("AlignActive", gamepad1.left_bumper);
            telemetry.addData("txMin", "%.2f", win[0]);
            telemetry.addData("txMax", "%.2f", win[1]);
            telemetry.addData("txTarget", "%.2f", targetTx);
            telemetry.addData("alignErr", "%.2f", txError);

            telemetry.addLine("---- No-Shot Zone ----");
            telemetry.addData("NO_SHOT_UNDER_IN", "%.2f", ShooterConfig.NO_SHOT_UNDER_IN);
            telemetry.addData("noShotZone", noShotZone);

            telemetry.addLine("---- Shooter Calc ----");
            telemetry.addData("USE_TABLE (match)", ShooterConfig.USE_TABLE);
            telemetry.addData("physicsTPS", "%.0f", shooter.getPhysicsTPS());
            telemetry.addData("tableTPS", "%.0f", shooter.getTableTPS());
            telemetry.addData("baseChosen", "%.0f", shooter.getBaseTPS());
            telemetry.addData("scale (physics only)", "%.3f", ShooterConfig.TPS_SCALE);
            telemetry.addData("offset (physics only)", "%.0f", ShooterConfig.TPS_OFFSET);
            telemetry.addData("finalTPS", "%.0f", shooter.getFinalTPS());

            telemetry.addLine("---- Shooter State ----");
            telemetry.addData("Armed", autoSpinArmed);
            telemetry.addData("Setpoint TPS", "%.0f", targetTPS);
            telemetry.addData("Actual TPS", "%.0f", robot.launchMotor.getVelocity());
            telemetry.addData("Err", "%.0f", (robot.launchMotor.getVelocity() - targetTPS));
            telemetry.addData("Ready", motorReady);
            telemetry.addData("Angle OK", angleOk);
            telemetry.addData("FeedAllowed", canShoot);
            telemetry.addData("Loaded", isLoaded);
            telemetry.addLine("TIP: Set NO_SHOT_UNDER_IN to your measured 'too close' distance.");
            telemetry.update();
        }
        
        robot.launchMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        logger.stop();
    }
}
