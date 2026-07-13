package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterStateLogic {
    private CRServo feedServo;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private CRServo intakeServo;
    private DistanceSensor rangeSensor;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        FIRST_ART,
        NEXT_ARTS,
        RESET;
    }
    private FlywheelState flywheelstate;


}
