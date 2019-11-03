package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name="test")
public class test extends LinearOpMode {
    @Override
    public void runOpMode() {
        robot_hardware robot = new robot_hardware(hardwareMap, telemetry);
        action_lib action = new action_lib(robot);

        waitForStart();
        while (opModeIsActive()) {
            float leftsticky = this.gamepad1.left_stick_y;
            float leftstickx = this.gamepad1.left_stick_x;
            float rightstickx = this.gamepad1.right_stick_x;
            robot.lf_drive.setPower(leftstickx - leftsticky + rightstickx);
            robot.rf_drive.setPower(-leftstickx - leftsticky - rightstickx);
            robot.lr_drive.setPower(-leftstickx - leftsticky + rightstickx);
            robot.rr_drive.setPower(leftstickx - leftsticky - rightstickx);
            if (this.gamepad1.left_bumper) {
                robot.intake1.setPower(1);
                robot.intake2.setPower(-1);
            } else if (this.gamepad1.right_bumper) {
                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);
            } else {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }
            if (this.gamepad1.a) {
                robot.claw1.setPosition(0.35);
            }
            if (this.gamepad1.x) {
                robot.claw1.setPosition(0.1);
            }
            if (this.gamepad1.y) {
                robot.arm.setPower(0.5);
            } else if (this.gamepad1.b) {
                robot.arm.setPower(-0.5);
            }
            else{
                robot.arm.setPower(0);
            }
            if(this.gamepad1.dpad_up){
                action.pick();
            }
            if(this.gamepad1.dpad_down) {
                action.yeeeeet();
            }
        }
    }
}
