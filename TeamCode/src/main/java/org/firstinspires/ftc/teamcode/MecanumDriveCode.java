package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MechanumDriveCode")
public class MecanumDriveCode extends LinearOpMode {
    robot_hardware hardware = new robot_hardware(hardwareMap, telemetry);

    @Override
    public void runOpMode() {
        robot_hardware robot = new robot_hardware(hardwareMap, telemetry);
        robot.claw2.setPosition(0.8);
        int speedCheck = 0;
        waitForStart();

        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;



            float arm_down = this.gamepad1.right_trigger;
            boolean arm_up = this.gamepad1.right_bumper;
            float intake = this.gamepad1.left_trigger;
            boolean outake = this.gamepad1.left_bumper;
            boolean grip = this.gamepad1.b;
            boolean gripRelease = this.gamepad1.a;
            boolean resetArm = this.gamepad1.x;
            boolean raiseArm = this.gamepad1.y;
            boolean speed = this.gamepad1.start;

            if (speed){
                sleep(200);
                if (speedCheck == 0) {
                    speedCheck = 1;
                } else if (speedCheck == 1) {
                    speedCheck = 0;
                    while(!speed){
                        break;
                    }
                }
            }

            if (speedCheck == 1) {
                strafe = strafe * 0.5;
                forward = forward * 0.5;
                turn = turn * 0.5;
            }else if(speedCheck == 0){
                strafe = -this.gamepad1.left_stick_x;
                forward = -this.gamepad1.left_stick_y;
                turn = -this.gamepad1.right_stick_x;
            }

            robot.lf_drive.setPower(strafe - forward + turn);
            robot.rf_drive.setPower(strafe + forward + turn);
            robot.lr_drive.setPower(-strafe - forward + turn);
            robot.rr_drive.setPower(-strafe + forward + turn);

            if (raiseArm) {
                if (robot.arm.getCurrentPosition() < 2000) {
                    while (robot.arm.getCurrentPosition() < 2000) robot.arm.setPower(1);
                    robot.arm.setPower(0);
                } else if (robot.arm.getCurrentPosition() > 2000) {
                    while (robot.arm.getCurrentPosition() > 2000) robot.arm.setPower(-1);
                    robot.arm.setPower(0);
                }
            }

            if (arm_down > 0) {
                robot.arm.setPower(-arm_down);
            } else {
                robot.arm.setPower(0);
            }
            if (arm_up) {
                robot.arm.setPower(1);
            } else {
                robot.arm.setPower(0);
            }
            if (intake > 0) {
                robot.intake_left.setPower(1);
                robot.intake_right.setPower(-1);
            } else if (outake) {
                robot.intake_left.setPower(-1);
                robot.intake_right.setPower(1);
            } else {
                robot.intake_left.setPower(0);
                robot.intake_right.setPower(0);
            }
            if (grip) {
                robot.claw2.setPosition(0.55);
            } else if (gripRelease) {
                robot.claw2.setPosition(0.8);
            }
            if (resetArm) {
                while (robot.arm.getCurrentPosition() > 10) robot.arm.setPower(-1);
                robot.arm.setPower(0);
            }
        }
    }
}