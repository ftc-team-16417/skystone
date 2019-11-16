package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mechanum")
public class Mechanum extends LinearOpMode {

    @Override
    public void runOpMode(){


        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);

        waitForStart();

        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;


            robot.lf_drive.setPower(strafe-forward+turn);
            robot.rf_drive.setPower(strafe+forward+turn);
            robot.lr_drive.setPower(-strafe-forward+turn);
            robot.rr_drive.setPower(-strafe+forward+turn);

//arm
                double grip1 = this.gamepad1.left_trigger;
                boolean grip2 = this.gamepad1.left_bumper;
                boolean arm_down = this.gamepad1.dpad_down;
                boolean arm_up = this.gamepad1.dpad_up;
                boolean intake = this.gamepad1.left_bumper;
                boolean outake = this.gamepad1.right_bumper;
                float grip_take = this.gamepad1.left_trigger;
                float grip_release = this.gamepad1.left_trigger;

                if(grip1 > 0.5) {
                    robot.grip.setPosition(0.25);
                }
                if(grip2) {
                    robot.grip.setPosition (0);
                }if (arm_down) {
                robot.arm.setTargetPosition(600);
                robot.arm.setPower(0.5);
            }else{
                    if(robot.arm.isBusy()){
                        return;
                    }
                    else{
                        robot.arm.setPower(0);
                    }
            }
            if (arm_up) {
                robot.arm.setTargetPosition(0);
                robot.arm.setPower(-0.5);
            }else{
                if(robot.arm.isBusy()){

                }
                else{
                    robot.arm.setPower(0);
                }
            }
            if(intake){
                robot.intake_left.setPower(1);
                robot.intake_right.setPower(-1);
            }
            else if (outake){
                robot.intake_left.setPower(-1);
                robot.intake_right.setPower(1);
            }else {
                robot.intake_left.setPower(0);
                robot.intake_right.setPower(0);
            }
            if(grip_take > 0){
                robot.claw1.setPosition(1);
            }
            if(grip_release>0){
                robot.claw1.setPosition(0);
            }



            }



        }

    }