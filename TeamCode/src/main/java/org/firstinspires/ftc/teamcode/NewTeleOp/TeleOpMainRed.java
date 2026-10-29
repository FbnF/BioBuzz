package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.LedService;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.LoggingService;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.RobotHardware;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.ShooterService;
import org.firstinspires.ftc.teamcode.NewTeleOp.Services.VisionService;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.utils.PDController;

// Main Red TeleOp
@Configurable
@TeleOp(name = "TeleOpMainRed", group = "TeleOp")
public class TeleOpMainRed extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private VisionService vision = VisionService.getInstance();
    private ShooterService shooter = ShooterService.getInstance();
    private LedService leds = LedService.getInstance();
    private LoggingService logger = LoggingService.getInstance();
    
    private Follower follower;
    private PDController alignController;
    private double speedFactor = 0.95; 

    public static boolean autoShooter = true;
    private boolean autoSpinArmed = true;
    private boolean prevG2DpadUp = false;
    private boolean prevG2DpadRight = false;
    private boolean prevG2DpadLeft = false;
    private boolean prevG2RT = false;
    private boolean prevG2LT = false;
    private boolean prevG2RB = false;
    private boolean prevG2LB = false;
    private boolean prevG2A = false;
    private boolean frontServoDown = false;

    private boolean isEjecting = false;
    private long ejectStartNs = 0;
    private double currentIntakePower = 0;
    private double currentOffset = ShooterConfig.RED_ALIGN_OFFSET;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision.init(hardwareMap);
        vision.reset();
        vision.setGoalTagIds(32, 31, 30, 34, 35, 36); // Red 6 goal tag IDs (33 and 37 filtered out)
        shooter.setIsBlue(false);
        robot.launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(500, 3, 0, 4));
        
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive(true);

        logger.init(hardwareMap, follower::getPose);
        alignController = new PDController(ShooterConfig.ALIGN_KP, ShooterConfig.ALIGN_KD);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            vision.update();
            leds.update();
            logger.record();

            // Drivetrain (GP1)
            if (gamepad1.a) speedFactor = 0.95; 
            if (gamepad1.b) speedFactor = 0.40;  
            if (gamepad1.x) speedFactor = 0.70;
            if (gamepad1.y) speedFactor = 1.25;

            double axial = -gamepad1.left_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x * speedFactor;
            double turn = -gamepad1.right_stick_x * speedFactor;

            // Auto align (GP1)
            double[] win = vision.getTxWindow();
            double txMin = win[0];
            double txMax = win[1];
            double tx = vision.getTx();
            double targetTx;

            if (tx < txMin || tx > txMax) {
                targetTx = (txMin + txMax) / 2.0;
            } else {
                targetTx = tx; // Already in window, set error to 0
            }

            double txError = (targetTx - tx) - currentOffset;

            if (ShooterConfig.AUTO_ALIGN_ENABLED && gamepad1.left_bumper
                    && vision.isVisionEnabled() && vision.isCurrentlySeeing()) {
                if (Math.abs(txError) <= ShooterConfig.ALIGN_ERR_DEADBAND_DEG) {
                    turn = 0;
                } else {
                    turn = alignController.update(txError);
                    // Apply minimum turn power to overcome static friction and lock onto target
                    if (Math.abs(turn) > 0 && Math.abs(turn) < ShooterConfig.ALIGN_MIN_TURN) {
                        turn = Math.copySign(ShooterConfig.ALIGN_MIN_TURN, turn);
                    }
                }
            } else {
                alignController.reset();
            }

            follower.setTeleOpDrive(axial, lateral, turn, true);

            // Shooter modes (GP2)
            if (gamepad2.dpad_up && !prevG2DpadUp) autoShooter = true;
            prevG2DpadUp = gamepad2.dpad_up;

            if (gamepad2.dpad_left && !prevG2DpadLeft) autoSpinArmed = !autoSpinArmed;
            prevG2DpadLeft = gamepad2.dpad_left;

            if (gamepad2.dpad_right && !prevG2DpadRight) vision.setVisionEnabled(!vision.isVisionEnabled());
            prevG2DpadRight = gamepad2.dpad_right;

            double targetTPS = 0;

            if (autoShooter) {
                if (autoSpinArmed) {
                    targetTPS = shooter.calculateVelocity(vision.getDistance(), vision.isTargetVisible(), vision.isCurrentlySeeing());
                    robot.launchMotor.setVelocity(targetTPS);
                } else {
                    robot.launchMotor.setPower(0);
                }
            } else {
                // Manual presets removed as per new specs
                robot.launchMotor.setPower(0);
            }

            // Subsystems
            boolean motorReady = Math.abs(robot.launchMotor.getVelocity() - targetTPS) < ShooterConfig.TPS_TOL;
            boolean isLoaded = robot.rangeSensor.getDistance(DistanceUnit.MM) < 170;
            boolean angleOk = vision.getTx() >= win[0] && vision.getTx() <= win[1];
            boolean noShotZone = vision.isNoShotZone();
            //Switching offsets
            if(vision.getDistance() >= ShooterConfig.DISTANCE_TO_SWITCH_TO_FAR){
                currentOffset = ShooterConfig.RED_ALIGN_FAR_OFFSET;
            } else{
                currentOffset = ShooterConfig.RED_ALIGN_OFFSET;
            }
            
            // Intake Latching logic (GP2)
            boolean rtPressed = gamepad2.right_trigger > 0.5;
            if (rtPressed && !prevG2RT) {
                currentIntakePower = 1.0;
            }
            prevG2RT = rtPressed;

            boolean ltPressed = gamepad2.left_trigger > 0.5;
            if (ltPressed && !prevG2LT) {
                currentIntakePower = 0.0;
                robot.feedServo.setPower(0);
            }
            prevG2LT = ltPressed;

            if (gamepad2.right_bumper && !prevG2RB) {
                currentIntakePower = -1.0;
            }
            prevG2RB = gamepad2.right_bumper;

            // Quick Eject burst (GP2)
            if (gamepad2.left_bumper && !prevG2LB && !isEjecting) {
                isEjecting = true;
                ejectStartNs = System.nanoTime();
                robot.intakeMotor.setPower(-0.7);
                robot.sideServo.setPower(0); // OUTTAKING: only intake motor runs
            }
            prevG2LB = gamepad2.left_bumper;

            if (isEjecting) {
                double elapsed = (System.nanoTime() - ejectStartNs) / 1e9;
                if (elapsed >= 0.1) {
                    isEjecting = false;
                    robot.intakeMotor.setPower(currentIntakePower);
                    // Sync side servo only for forward intake
                    robot.sideServo.setPower(currentIntakePower > 0 ? currentIntakePower : 0);
                }
            } else {
                robot.intakeMotor.setPower(currentIntakePower);
                // Sync side servo only for forward intake (clears fourth ball in reverse)
                robot.sideServo.setPower(currentIntakePower > 0 ? currentIntakePower : 0);
            }

            // Feed Servo overrides (GP2)
            if (gamepad2.y) {
                robot.feedServo.setPower(shooter.getFeedPower(vision.getDistance()));
            } else if (gamepad2.x) {
                robot.feedServo.setPower(0);
            } else if (!isEjecting) {
                if (!motorReady && !isLoaded) robot.feedServo.setPower(0);
            }

            // Toggle frontServo position up (0.06) / down (0.38) with Gamepad 2 A button
            boolean aPressed = gamepad2.a;
            if (aPressed && !prevG2A) {
                frontServoDown = !frontServoDown;
            }
            prevG2A = aPressed;

            double frontPos = frontServoDown ? ShooterConfig.FRONT_SERVO_FLOWER_POS : ShooterConfig.FRONT_SERVO_INIT_POS;
            robot.frontServo.setPosition(frontPos);

            // Sync intakeHelper and flowerServo (flowerServo runs opposite to intakeHelper only when down and intaking)
            double helperPower = 0.0;
            double flowerPower = 0.0;
            if (isEjecting || currentIntakePower > 0.0) {
                helperPower = ShooterConfig.INTAKE_HELPER_INTAKE_POWER;
                if (frontServoDown) {
                    flowerPower = -helperPower;
                }
            } else if (currentIntakePower < 0.0) {
                helperPower = ShooterConfig.INTAKE_HELPER_REVERSE_POWER;
                if (frontServoDown) {
                    flowerPower = -helperPower;
                }
            }
            robot.intakeHelper.setPower(helperPower);
            robot.flowerServo.setPower(flowerPower);

            // Sync blockServo with cam-jet (feedServo)
            if (robot.feedServo.getPower() != 0.0) {
                robot.blockServo.setPosition(0.38); // go
            } else {
                robot.blockServo.setPosition(0.26); // block
            }

            leds.setStatus(vision.isTargetVisible(), motorReady, noShotZone);

            // Telemetry
            telemetry.addLine("---- Modes ----");
            telemetry.addData("Shooter Mode", autoShooter ? "AUTO" : "OFF");
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
