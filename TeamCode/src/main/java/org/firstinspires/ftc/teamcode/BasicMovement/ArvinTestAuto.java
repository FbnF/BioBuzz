package org.firstinspires.ftc.teamcode.BasicMovement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "ArvinTestAuto", group = "MotorTests")

public class ArvinTestAuto extends LinearOpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;

    @Override
    public void runOpMode() {
        //components
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left motors so positive power = forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        waitForStart();

        //Autonomous path
        moveForward(1300, 0.5);
        turnRight(200, 0.5);
        moveForward(2400, 0.5);
        turnRight(550, 0.5);
        moveForward(8200, 1.0);
        turnLeft(700, 0.5);
        moveForward(7400, 0.5);
        moveBackward(7400, 0.5);
        turnRight(775, 0.5);
        moveBackward(8250, 1.0);
        turnLeft(700, 0.5);
        moveBackward(2400, 0.5);
        turnLeft(200, 0.5);
        moveBackward(1300, 0.5);

    }

    public void moveForward(int ticks, double speed) {
        setTargetPosition(ticks, ticks, ticks, ticks);
        setRunToPosition(speed);
    }

    public void moveBackward(int ticks, double speed) {
        setTargetPosition(-ticks, -ticks, -ticks, -ticks);
        setRunToPosition(speed);
    }

    public void turnLeft(int ticks, double speed) {
        // Left side backwards, right side forwards
        setTargetPosition(-ticks, ticks, -ticks, ticks);
        setRunToPosition(speed);
    }

    public void turnRight(int ticks, double speed) {
        // Right side backwards, left side forwards
        setTargetPosition(ticks, -ticks, ticks, -ticks);
        setRunToPosition(speed);
    }

    private void setTargetPosition(int lf, int rf, int lb, int rb) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rf);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + lb);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rb);
    }

    private void setRunToPosition(double speed) {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && (leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy())) {
            telemetry.addData("Status", "Driving to target");
            telemetry.addData("Speed: ", speed);
            telemetry.addData("LF Pos: ", "%d / %d", leftFront.getCurrentPosition(), leftFront.getTargetPosition());
            telemetry.addData("RF Pos: ", "%d / %d", rightFront.getCurrentPosition(), rightFront.getTargetPosition());
            telemetry.addData("LB Pos: ", "%d / %d", leftBack.getCurrentPosition(), leftBack.getTargetPosition());
            telemetry.addData("RB Pos: ", "%d / %d", rightBack.getCurrentPosition(), rightBack.getTargetPosition());
            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Reset to normal run mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
