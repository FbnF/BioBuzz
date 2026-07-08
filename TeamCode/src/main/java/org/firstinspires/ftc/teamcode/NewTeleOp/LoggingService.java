package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.TinyCsvLoggerFlex;
import com.pedropathing.geometry.Pose;
import java.util.function.Supplier;

/**
 * LoggingService - High-fidelity flight recorder.
 */
public class LoggingService {

    private static LoggingService instance = null;
    private TinyCsvLoggerFlex logger;
    private boolean enabled = true;

    public static LoggingService getInstance() {
        if (instance == null) {
            instance = new LoggingService();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap, Supplier<Pose> poseSupplier) {
        if (!enabled) return;

        RobotHardware robot = RobotHardware.getInstance();
        VisionService vision = VisionService.getInstance();
        ShooterService shooter = ShooterService.getInstance();

        logger = TinyCsvLoggerFlex.create(
                hardwareMap,
                "teleop_new",
                TinyCsvLoggerFlex.doubleCol("dist_filt", vision::getDistance),
                TinyCsvLoggerFlex.doubleCol("tx", vision::getTx),
                TinyCsvLoggerFlex.doubleCol("target_tps", () -> shooter.calculateVelocity(vision.getDistance(), vision.isTargetVisible())),
                TinyCsvLoggerFlex.motorEx("launch", robot.launchMotor),
                TinyCsvLoggerFlex.motorEx("intake", robot.intakeMotor),
                TinyCsvLoggerFlex.pedroPose("pose", poseSupplier)
        );
    }

    public void record() {
        if (enabled && logger != null) {
            logger.record("run");
        }
    }

    public void stop() {
        if (logger != null) {
            logger.close();
        }
    }
}
