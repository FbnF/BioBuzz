package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.utils.PDController;

// Main Red TeleOp
@TeleOp(name = "TeleOpMainRed", group = "TeleOp")
public class TeleOpMainRed extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private VisionService vision = VisionService.getInstance();
    private ShooterService shooter = ShooterService.getInstance();
    private LedService leds = LedService.getInstance();
    private LoggingService logger = LoggingService.getInstance();
    
    private Follower follower;
    private PDController alignController;
    private double speedFactor = 1.2; 
    
    private boolean autoShooter = true;
    private boolean autoSpinArmed = true;
    private boolean prevG2DpadUp = false;
    private boolean prevG2DpadRight = false;
    private boolean prevG2DpadLeft = false;
    private boolean isEjecting = false;
    private long ejectStartNs = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision.init(hardwareMap);
        vision.setGoalTagId(24); 
        shooter.setIsBlue(false);
        
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

            // Drivetrain
            if (gamepad1.a) speedFactor = 1.35; 
            if (gamepad1.b) speedFactor = 0.4;  
            if (gamepad1.x) speedFactor = 0.7;  

            double axial = -gamepad1.right_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x * speedFactor;
            double turn = -gamepad1.right_stick_x * speedFactor;

            // Vision toggle
            if (gamepad2.dpad_left && !prevG2DpadLeft) {
                vision.setVisionEnabled(!vision.isVisionEnabled());
            }
            prevG2DpadLeft = gamepad2.dpad_left;

            // Auto align
            double[] win = vision.getTxWindow();
            double targetTx = (win[0] + win[1]) / 2.0;
            double txError = targetTx - vision.getTx();

            if (gamepad1.left_bumper && vision.isTargetVisible()) {
                turn = alignController.update(txError);
            } else {
                alignController.reset();
            }

            follower.setTeleOpDrive(axial, lateral, turn, true);

            // Shooter modes
            if (gamepad2.dpad_up && !prevG2DpadUp) autoShooter = !autoShooter;
            prevG2DpadUp = gamepad2.dpad_up;

            if (gamepad2.dpad_right && !prevG2DpadRight) autoSpinArmed = !autoSpinArmed;
            prevG2DpadRight = gamepad2.dpad_right;

            double targetTPS = 0;

            if (autoShooter) {
                if (autoSpinArmed) {
                    targetTPS = shooter.calculateVelocity(vision.getDistance(), vision.isTargetVisible());
                    robot.launchMotor.setVelocity(targetTPS);
                } else {
                    robot.launchMotor.setPower(0);
                }
            } else {
                // Manual presets
                if (gamepad2.a) targetTPS = ShooterConfig.SHOOTER_VEL_LONG; 
                else if (gamepad2.b) targetTPS = 1500.0;                   
                else if (gamepad2.left_bumper) targetTPS = ShooterConfig.SHOOTER_VEL_SHORT; 
                else if (gamepad2.x) targetTPS = 0;                        

                if (targetTPS > 0) robot.launchMotor.setVelocity(targetTPS);
                else robot.launchMotor.setPower(0);
            }

            // Subsystems
            boolean motorReady = Math.abs(robot.launchMotor.getVelocity() - targetTPS) < ShooterConfig.TPS_TOL;
            boolean isLoaded = robot.rangeSensor.getDistance(DistanceUnit.MM) < 170;
            boolean angleOk = vision.getTx() >= win[0] && vision.getTx() <= win[1];
            boolean noShotZone = vision.isNoShotZone();
            
            boolean canShoot = motorReady && (autoShooter ? (autoSpinArmed && vision.isTargetVisible() && angleOk && !noShotZone) : true);

            if (gamepad2.y) {
                if (canShoot) {
                    double dist = vision.getDistance();
                    robot.feedServo.setPower(shooter.getFeedPower(dist));
                    robot.intakeMotor.setPower(1.0);
                    robot.sideServo.setPower(shooter.getSidePower(dist));
                } else {
                    leds.triggerWarning();
                }
            } else if (gamepad2.right_trigger > 0) {
                robot.intakeMotor.setPower(1.0);
                robot.sideServo.setPower(1.0);
            } else if (gamepad2.right_bumper) {
                robot.intakeMotor.setPower(-1.0);
                robot.sideServo.setPower(-1.0);
            } else if (gamepad2.left_bumper && autoShooter && !isEjecting) {
                isEjecting = true;
                ejectStartNs = System.nanoTime();
                robot.intakeMotor.setPower(-0.7);
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

            // Telemetry
            telemetry.addLine("---- Modes ----");
            telemetry.addData("Shooter Mode", autoShooter ? "AUTO" : "MANUAL");
            telemetry.addData("Armed (Auto)", autoSpinArmed);
            telemetry.addData("Speed Factor", speedFactor);

            telemetry.addLine("---- Vision ----");
            telemetry.addData("Visible", vision.isTargetVisible());
            telemetry.addData("Dist (In)", "%.1f", vision.getDistance());
            telemetry.addData("Tx", "%.2f", vision.getTx());
            telemetry.addData("No-Shot", noShotZone);

            telemetry.addLine("---- Shooter ----");
            telemetry.addData("Target TPS", "%.0f", targetTPS);
            telemetry.addData("Actual TPS", "%.0f", robot.launchMotor.getVelocity());
            telemetry.addData("Ready", motorReady);
            telemetry.addData("Loaded", isLoaded);
            telemetry.update();
        }
        
        robot.launchMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        logger.stop();
    }
}
