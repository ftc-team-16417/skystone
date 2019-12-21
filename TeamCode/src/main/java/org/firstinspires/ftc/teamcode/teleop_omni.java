package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="teleop omni")
public class teleop_omni extends LinearOpMode {
    @Override
    public void runOpMode() {
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

        while (opModeIsActive()) {

            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double cwRotate = gamepad1.right_stick_x;

            boolean intake = gamepad1.right_bumper;
            boolean outake = gamepad1.left_bumper;

            boolean openClaw = gamepad1.dpad_left;
            boolean closeClaw = gamepad1.dpad_right;

            boolean armup = gamepad1.dpad_up;
            boolean armdown = gamepad1.dpad_down;

            boolean clampDown = gamepad1.x;
            boolean clampUp = gamepad1.b;
            robot.lf_drive.setPower(sideways-forward+cwRotate);
            robot.rf_drive.setPower(-sideways-forward-cwRotate);
            robot.lr_drive.setPower(-sideways-forward+cwRotate);
            robot.rr_drive.setPower(sideways-forward-cwRotate);

            if(intake == true){
                robot.left_intake.setPower(1);
                robot.right_intake.setPower(-1);

            }
            else if(outake == true){
                robot.left_intake.setPower(-1);
                robot.right_intake.setPower(1);
            }

            if(intake == false){
                robot.left_intake.setPower(0);
                robot.right_intake.setPower(0);

            }
            else if(outake == false){
                robot.left_intake.setPower(0);
                robot.right_intake.setPower(0);
            }
            
            if (openClaw == true)
                robot.claw1.setPosition(0.5);
            else if (closeClaw == true)
                robot.claw1.setPosition(0.3);




            if(armup == true){
                robot.arm.setPower(1);
            }
            else if(armdown == true){
                robot.arm.setPower(-1);;
            }
            if(armup == false){
                robot.arm.setPower(0);
            }
            else if(armdown == false){
                robot.arm.setPower(0);
            }






            //X FOR CLAMPING DOWN
            if (clampDown == true) {
                robot.grab1.setPosition(0.5);
                robot.grab2.setPosition(-0.5);
                while (robot.grab1.getPosition() < 0.5) {
                    robot.grab1.setPosition(0.08);
                    robot.grab2.setPosition(0.53);
                    while (robot.grab1.getPosition() > 0.1) {
                        //idle
                    }
                }
                clampDown = false;
            }
            //B FOR CLAMPING UP
            else if (clampUp == true) {
                robot.grab1.setPosition(0.2);
                robot.grab2.setPosition(-0.2);
                while (robot.grab1.getPosition() > 0.2) {
                    robot.grab1.setPosition(0.35);
                    robot.grab2.setPosition(0.25);
                    while (robot.grab1.getPosition() < 0.3) {
                        //idle
                    }
                    clampUp = false;
                }
            }

        }
    }
}
