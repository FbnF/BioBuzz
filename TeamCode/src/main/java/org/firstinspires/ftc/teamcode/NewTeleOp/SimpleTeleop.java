package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "SimpleTeleop", group = "TeleOp")
public class SimpleTeleop extends LinearOpMode {

    private Follower follower;
    private double speedFactor = 0.70; 

    @Override
    public void runOpMode() {
        // Initialize the Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        
        // Start TeleOp drive mode with brake mode enabled
        follower.startTeleopDrive(true);

        telemetry.addLine("SimpleTeleop Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // Speed Factor Selection (GP1)
            if (gamepad1.a) speedFactor = 0.95; 
            if (gamepad1.b) speedFactor = 0.40;  
            if (gamepad1.x) speedFactor = 0.70;
            if (gamepad1.y) speedFactor = 1.25;

            // Map sticks to axial, lateral, and turn
            double axial = -gamepad1.right_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x * speedFactor;
            double turn = -gamepad1.right_stick_x * speedFactor;

            // Set the drive powers (Robot Centric)
            follower.setTeleOpDrive(axial, lateral, turn, true);

            // Telemetry for the drivers
            telemetry.addData("Speed Factor", speedFactor);
            telemetry.addData("Axial", "%.2f", axial);
            telemetry.addData("Lateral", "%.2f", lateral);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.update();
        }
    }
}
