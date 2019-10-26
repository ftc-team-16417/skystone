package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class test extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);

        waitForStart();
        while(opModeIsActive()){
            float leftsticky = this.gamepad1.left_stick_y;
            float leftstickx = this.gamepad1.left_stick_x;
            float rightstickx = this.gamepad1.right_stick_x;
            robot.lf_drive.setPower(leftstickx+leftsticky+rightstickx);
            robot.rf_drive.setPower(-leftstickx+leftsticky-rightstickx);
            robot.lr_drive.setPower(-leftstickx+leftsticky+rightstickx);
            robot.rr_drive.setPower(leftstickx+leftsticky-rightstickx);
        }
    }
}
