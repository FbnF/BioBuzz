package org.firstinspires.ftc.teamcode.BasicMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "ArvinMotor", group = "MotorTests") //don't forget to rename

public class ArvinMotor extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;
    @Override
    public void runOpMode() { //makes ArvinMotor not give error
        //initialize my components
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setPower(0);
        motor2.setPower(0);
        waitForStart();
        while (opModeIsActive()) { //wait i actually understand what this is saying, woohoo
            //logic stuff
            if(gamepad1.a == true) {
                motor1.setPower(0.25);
                motor2.setPower(1.0);
            }
            else if(gamepad1.b) {
                motor1.setPower(0.5);
                motor2.setPower(0.5);
            }
            else if(gamepad1.y) {
                motor1.setPower(1.0);
                motor2.setPower(0.25);
            }
            else {
                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }
}
