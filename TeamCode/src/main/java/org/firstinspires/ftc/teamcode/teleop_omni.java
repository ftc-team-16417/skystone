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
        robot.lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){

            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double cwRotate = gamepad1.right_stick_x;
            boolean armdown = gamepad1.left_bumper;
            boolean armup = gamepad1.right_bumper;
            robot.lf_drive.setPower(forward-sideways-cwRotate);
            robot.lr_drive.setPower(forward+sideways-cwRotate);
            robot.rf_drive.setPower(-forward-sideways-cwRotate);
            robot.rr_drive.setPower(-forward+sideways-cwRotate);
            if(armdown == true) {
                while (armdown == true) {
                    robot.arm.setPower(0.5);
                    if(armdown == false){
                        robot.arm.setPower(0);
                        break;
                    }

                }

            }
            else if(armup == true){
                while(armup == true) {
                    robot.arm.setPower(-0.5);
                    if(armup == false){
                        robot.arm.setPower(0);
                        break;
                    }
                }
            }
        }
    }
}
