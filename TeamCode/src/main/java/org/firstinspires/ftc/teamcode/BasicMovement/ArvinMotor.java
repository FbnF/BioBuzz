package org.firstinspires.ftc.teamcode.BasicMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "ArvinMotor", group = "MotorTests")
public class ArvinMotor extends LinearOpMode {
    DcMotor rightFront;
    DcMotor rightBack;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setPower(0);
        rightBack.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            double leftStickVal = -gamepad1.left_stick_y;
            double rightStickVal = -gamepad1.right_stick_y;

            rightFront.setPower(leftStickVal);
            rightBack.setPower(rightStickVal);

            telemetry.addData("RightFront Power", leftStickVal);
            telemetry.addData("RightRear Power", rightStickVal);
            telemetry.update();
        }
    }
}
