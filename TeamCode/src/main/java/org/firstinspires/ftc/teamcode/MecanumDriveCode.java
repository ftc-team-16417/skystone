package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MechanumDriveCode")
public class MecanumDriveCode extends LinearOpMode {
    robot_hardware hardware = new robot_hardware(hardwareMap, telemetry);
    @Override
    public void runOpMode() {
        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
        robot.claw2.setPosition(-0.5);
        waitForStart();

        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;
            int checkGrip = 0;

            robot.lf_drive.setPower(strafe-forward+turn);
            robot.rf_drive.setPower(strafe+forward+turn);
            robot.lr_drive.setPower(-strafe-forward+turn);
            robot.rr_drive.setPower(-strafe+forward+turn);

            float arm_down = this.gamepad1.right_trigger;
            boolean arm_up = this.gamepad1.right_bumper;
            float intake = this.gamepad1.left_trigger;
            boolean outake = this.gamepad1.left_bumper;
            boolean grip = this.gamepad1.b;
            boolean resetArm = this.gamepad1.x;

            if (arm_down>0){
                robot.arm.setPower(-arm_down);
            }else{
                robot.arm.setPower(0);
            }
            if (arm_up) {
                robot.arm.setPower(1);
            }else{
                robot.arm.setPower(0);
            }

            if(intake>0){
                robot.intake_left.setPower(1);
                robot.intake_right.setPower(-1);
            }else if(outake){
                robot.intake_left.setPower(-1);
                robot.intake_right.setPower(1);
            }else{
                robot.intake_left.setPower(0);
                robot.intake_right.setPower(0);
            }

            if(grip){
                if(checkGrip == 1){
                    robot.claw2.setPosition(0);
                    checkGrip = 0;
                }else if(checkGrip == 0){
                    robot.claw2.setPosition(-4);
                }

            }

            if(resetArm){
                while(robot.arm.getCurrentPosition()>10)robot.arm.setPower(-1);
                robot.arm.setPower(0);
            }




        }
    }
}