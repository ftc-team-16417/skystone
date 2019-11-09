package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "sam")
public class review extends LinearOpMode {

    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap,telemetry);
        waitForStart();
        while(true) {
            double forward = this.gamepad1.right_stick_x;
            double turn = this.gamepad1.right_stick_x;
            double strafe = this.gamepad1.left_stick_y;
            robot.lf_drive.setPower(forward - turn + strafe);
            robot.rf_drive.setPower(-forward - turn + strafe);
            robot.rr_drive.setPower(-forward + turn - strafe);
            robot.lr_drive.setPower(forward + turn - strafe);


        }
    }
}
