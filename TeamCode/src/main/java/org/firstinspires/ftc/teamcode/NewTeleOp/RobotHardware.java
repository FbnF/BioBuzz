package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;

/**
 * Master hardware map for all subsystems.
 * If a device is missing, the robot will throw an error on init so we know immediately.
 */
public class RobotHardware {

    public DcMotorEx intakeMotor;
    public DcMotorEx launchMotor;
    public CRServo feedServo;
    public CRServo sideServo;
    public DistanceSensor rangeSensor;
    public VoltageSensor battery;
    public Servo puckLight;

    private static RobotHardware instance = null;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        // Basic mapping using HardwareConfig names
        intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareConfig.INTAKE_MOTOR);
        launchMotor = hardwareMap.get(DcMotorEx.class, HardwareConfig.SHOOTER_MOTOR);
        feedServo = hardwareMap.get(CRServo.class, HardwareConfig.FEED_SERVO);
        sideServo = hardwareMap.get(CRServo.class, HardwareConfig.SIDE_SERVO);
        rangeSensor = hardwareMap.get(DistanceSensor.class, HardwareConfig.RANGE_SENSOR);
        puckLight = hardwareMap.get(Servo.class, "PuckLight");
        battery = hardwareMap.voltageSensor.iterator().next();

        // Standard motor behaviors
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset positions/powers
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);
        feedServo.setPower(0.0);
        sideServo.setPower(0.0);
        puckLight.setPosition(0.0);
    }
}
