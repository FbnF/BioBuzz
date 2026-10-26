package org.firstinspires.ftc.teamcode.BasicMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "ArvinServo", group = "MotorTests")
public class ArvinServo extends LinearOpMode {
    CRServo intakeHelper;

    @Override
    public void runOpMode() {
        intakeHelper = hardwareMap.get(CRServo.class, "intakeHelper");

        intakeHelper.setPower(0.0);

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.0;
            if (gamepad1.a) {
                power = 1.0;
            } else if (gamepad1.b) {
                power = -1.0;
            } else if (gamepad1.x) {
                power = 0.5;
            } else if (gamepad1.y) {
                power = -0.5;
            }

            intakeHelper.setPower(power);

            telemetry.addData("Testing intakeHelper Power", power);
            telemetry.update();
        }
    }
}
