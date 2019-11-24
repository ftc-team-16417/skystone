package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="teleop omni")
public class teleop_omni extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap, telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot, action);
        waitForStart();

        while(opModeIsActive()){
            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double cwRotate = gamepad1.right_stick_x;
            this.robot.lf_drive.setPower(forward-sideways+cwRotate);
            this.robot.lr_drive.setPower(forward+sideways+cwRotate);
            this.robot.rf_drive.setPower(-forward-sideways+cwRotate);
            this.robot.rr_drive.setPower(-forward+sideways+cwRotate);
        }
    }
}
