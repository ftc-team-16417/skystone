package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous_omni extends LinearOpMode {


    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        action_lib action = null;
        imu_lib imu = null;
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("lf_drive", robot.lf_drive.getCurrentPosition());
            telemetry.addData("lr_drive", robot.lf_drive.getCurrentPosition());
            telemetry.addData("rf_drive", robot.lf_drive.getCurrentPosition());
            telemetry.addData("rr_drive", robot.lf_drive.getCurrentPosition());

        }

    }
}
