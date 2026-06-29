package org.firstinspires.ftc.teamcode.BasicMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Arvin Servo", group = "MotorTests")

public class ArvinServo extends LinearOpMode {
    Servo myServo;

    @Override
    public void runOpMode() {
        myServo = hardwareMap.get(Servo.class, "servo");
        myServo.setPosition(10.0);

        waitForStart();

        boolean lastA = false; //memory of last state of button
        boolean toggle = false; //memory of servo open or close
        while (opModeIsActive()) {
            //logic stuff
            //check for button press
            if (gamepad1.a && !lastA) {
                toggle = !toggle; //flip between true false each press
            }
            if (toggle) {
                myServo.setPosition(1.0); //open
            }
            else {
                myServo.setPosition(0.0); //closed
            }
            telemetry.addData("Servo Position: ", myServo.getPosition());
            telemetry.update();
        }
    }
}
