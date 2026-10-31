package org.firstinspires.ftc.teamcode.BiobuzzAuto.Services;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

public class initiService {
    public void init(Follower follower, DcMotor intake, DcMotorEx shooter, CRServo feed, CRServo sideServo, CRServo intakeHelper, CRServo flowerServo, Servo frontServo, Servo blockServo){
                feed.setPower(-0.9);
                blockServo.setPosition(0.38);
                intake.setPower(ShooterConfig.INTAKE_POWER);
                sideServo.setPower(1.0);
                intakeHelper.setPower(ShooterConfig.INTAKE_HELPER_INTAKE_POWER);
                frontServo.setPosition(ShooterConfig.FRONT_SERVO_FLOWER_POS);
                flowerServo.setPower(1.0);
    }
}
