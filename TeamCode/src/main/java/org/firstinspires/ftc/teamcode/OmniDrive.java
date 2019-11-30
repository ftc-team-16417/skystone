package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "omni_drive")
public class OmniDrive extends LinearOpMode {

    @Override
    public void runOpMode() {


        robot_hardware robot = new robot_hardware(hardwareMap, telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot, action);
        waitForStart();
        double prev = imu.getAngle();
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setPower(-0.2);
        sleep(1100);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = (-this.gamepad1.right_stick_x) / 2;

            if (Math.abs(forward) > 0.5 && Math.abs(strafe) < 0.1 && Math.abs(turn) < 0.1) {


                    double correction=0;
                    // double correction = imu.getProportionalTerm(prev,imu.getAngle(),0.08,0);

                    robot.lf_drive.setPower(-forward + correction);
                    robot.rf_drive.setPower(forward + correction);
                    robot.lr_drive.setPower(-forward + correction);
                    robot.rr_drive.setPower(forward + correction);

                } else {
                    prev = imu.getAngle();
                    robot.lf_drive.setPower(strafe - forward + turn);
                    robot.rf_drive.setPower(strafe + forward + turn);
                    robot.lr_drive.setPower(-strafe - forward + turn);
                    robot.rr_drive.setPower(-strafe + forward + turn);
                }
                //Outputing previous angle
                telemetry.addData("previous angle ", prev);
                telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
                telemetry.update();
                //arm

                double grip2 = this.gamepad2.right_trigger;
                double grip1 = this.gamepad1.left_trigger;
                boolean arm_down = this.gamepad1.dpad_up;
                boolean arm_up = this.gamepad1.dpad_down;
                boolean intake = this.gamepad1.left_bumper;
                boolean outake = this.gamepad1.right_bumper;
                boolean rotate1 = this.gamepad1.a;
                boolean rotate2 = this.gamepad1.b;
                boolean grab_up = this.gamepad1.y;
                boolean grab_down = this.gamepad1.start;
                //--------------IN PROGRESS ---------------

                boolean position2 = this.gamepad1.x;
                //---------------Coming Soon!--------------
                if (position2) {
                    robot.arm.setTargetPosition(700);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(0.5);
                    while(robot.arm.isBusy());
                    robot.arm.setPower(0);
                }

                if (grip1 > 0.5) {
                    robot.claw1.setPosition(0.45);
                }
                if (grip2 > 0.5) {
                    robot.claw1.setPosition(0.3);
                }
                if (arm_down) {
                    robot.arm.setTargetPosition(4000);
                    robot.arm.setPower(0.6);
                } else {
                    if (robot.arm.isBusy()) {
                    } else {
                        robot.arm.setPower(0);
                    }
                }
                if (arm_up) {
                    robot.arm.setTargetPosition(0);
                    robot.arm.setPower(-0.5);
                } else {
                    if (robot.arm.isBusy()) {

                    } else {
                        robot.arm.setPower(0);
                    }
                }
                if (intake) {
                    robot.intake_left.setPower(1);
                    robot.intake_right.setPower(-1);
                } else if (outake) {
                    robot.intake_left.setPower(-1);
                    robot.intake_right.setPower(1);
                } else {
                    robot.intake_left.setPower(0);
                    robot.intake_right.setPower(0);
                }
                if (rotate1) {
                    robot.claw2.setPosition(0.35);
                }
                if (rotate2) {
                    robot.claw2.setPosition(0.9);
                }
                if (grab_up) {
                    action.grab(0);
                }
                if (grab_down){
                    action.grab(0.2);
                }

            }


        }

    }
