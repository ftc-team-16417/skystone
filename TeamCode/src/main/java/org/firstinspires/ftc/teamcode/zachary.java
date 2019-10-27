
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * this is a very simple teleop code and should not be used for competition
 */
@TeleOp(name = "zac")
public class zachary extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap,telemetry);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//hold the arm up
        telemetry.addLine("done calibration, ready to start");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            float leftsticky = -this.gamepad1.left_stick_y;
            float leftstickx = this.gamepad1.left_stick_x;
            float rightstickx = this.gamepad1.right_stick_x;
            boolean intakeBtn = this.gamepad1.left_bumper;
            boolean outakeBtn = this.gamepad1.right_bumper;
            boolean armup = this.gamepad1.a;
            boolean armdown = this.gamepad1.b;
            boolean clawclose = this.gamepad1.x;
            boolean clawopen = this.gamepad1.y;
            robot.lf_drive.setPower(leftstickx+leftsticky+rightstickx);
            robot.rf_drive.setPower(-leftstickx+leftsticky-rightstickx);
            robot.lr_drive.setPower(-leftstickx+leftsticky+rightstickx);
            robot.rr_drive.setPower(leftstickx+leftsticky-rightstickx);
            if(intakeBtn) {
                robot.right_intake.setPower(1);
                robot.left_intake.setPower(-1);
            }else if(outakeBtn){
                robot.right_intake.setPower(-1);
                robot.left_intake.setPower(1);
            }else{
                robot.right_intake.setPower(0);
                robot.left_intake.setPower(0);
            }
            if(armup){
                robot.arm.setPower(-1);
            }else if(armdown){
                robot.arm.setPower(1);
            }else{
                robot.arm.setPower(0);
            }
            if(clawclose == true){
                robot.claw1.setPosition(0.1);
            }
            else if(clawopen == true){
                robot.claw1.setPosition(0.35);
            }
            if(this.gamepad1.dpad_up){
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setTargetPosition(1000);
                robot.arm.setPower(0.5);
                while(robot.arm.isBusy());
                robot.arm.setPower(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else if(this.gamepad1.dpad_down){
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setTargetPosition(1500);
                robot.arm.setPower(0.5);
                while(robot.arm.isBusy());
                robot.arm.setPower(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (this.gamepad1.dpad_left){
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setTargetPosition(750);
                robot.claw1.setPosition(0.35);
                robot.arm.setTargetPosition(1400);
                robot.arm.setPower(0.5);
                while(robot.arm.isBusy()){
                    telemetry.addData("position",robot.arm.getCurrentPosition());
                    telemetry.update();
                }
                robot.arm.setPower(0);
                robot.claw1.setPosition(0.1);
                sleep(500);
                robot.arm.setTargetPosition(750);
                robot.arm.setPower(0.5);
                while(robot.arm.isBusy()){
                    telemetry.addData("position",robot.arm.getCurrentPosition());
                    telemetry.update();
                }
                robot.arm.setPower(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (this.gamepad1.dpad_right){
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.5);
                robot.arm.setTargetPosition(1300);
                while(robot.arm.isBusy()){
                    telemetry.addData("position",robot.arm.getCurrentPosition());
                    telemetry.update();
                robot.claw1.setPosition(0.35);


                robot.right_intake.setPower(1);
                robot.left_intake.setPower(-1);

            }
            telemetry.addData("arm position",robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}