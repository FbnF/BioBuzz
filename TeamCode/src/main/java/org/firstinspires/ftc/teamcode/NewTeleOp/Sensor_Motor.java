package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.NewTeleOp.Services.RobotHardware;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

/**
 * simple TeleOp to test the flywheel at specific speeds.
 * A: 720 TPS
 * B: Short Velocity
 * X: Long Velocity
 * Y: Stop
 */
@TeleOp(name = "Motor_testing", group = "Sensor")
public class Sensor_Motor extends LinearOpMode {

    private DcMotorEx motor;
    double targetTPS;

    @Override
    public void runOpMode() {
        // "motor" must match the name you gave this motor in the
        // Robot Controller's Configure Robot Hardware screen.
        motor = hardwareMap.get(DcMotorEx.class, "motor");


        telemetry.addLine("Ready - press play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Left stick up = negative value, so we flip the sign
            // to make "up" drive the motor forward.
            if (gamepad1.a) targetTPS = 1300;
            if (gamepad1.b) targetTPS = 1325;
            if (gamepad1.x) targetTPS = 1350;
            if (gamepad1.y) targetTPS = 1400;
            if (gamepad1.right_bumper) targetTPS=1375;
            if (gamepad1.left_bumper) targetTPS=1325;



            motor.setVelocity(targetTPS);


        }

    }
}
