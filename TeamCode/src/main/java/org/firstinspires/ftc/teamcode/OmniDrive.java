package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "omni_drive")
public class OmniDrive extends LinearOpMode {

    @Override
    public void runOpMode(){


        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot,action);
        waitForStart();
        double prev = imu.getAngle();
        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = (-this.gamepad1.right_stick_x)/2;

            if(Math.abs(forward)>0.5&& Math.abs(strafe)<0.1&&Math.abs(turn)<0.1){
                double correction = 0;//.imu.getProportionalTerm(prev,0.02,0);
                robot.lf_drive.setPower(-forward+correction);
                robot.rf_drive.setPower(+forward+correction);
                robot.lr_drive.setPower(-forward+correction);
                robot.rr_drive.setPower(+forward+correction);


            }
            else {
                prev = imu.getAngle();
                robot.lf_drive.setPower(strafe - forward + turn);
                robot.rf_drive.setPower(strafe + forward + turn);
                robot.lr_drive.setPower(-strafe - forward + turn);
                robot.rr_drive.setPower(-strafe + forward + turn);
            }
            telemetry.addData("previous angle ", prev);
            telemetry.update();
            //arm
            double grip1 = this.gamepad1.left_trigger;
            double grip2 = this.gamepad1.right_trigger;
            boolean arm_down = this.gamepad1.dpad_up;
            boolean arm_up = this.gamepad1.dpad_down;
            boolean intake = this.gamepad1.left_bumper;
            boolean outake = this.gamepad1.right_bumper;
            boolean rotate1 = this.gamepad1.a;
            boolean rotate2 = this.gamepad1.b;
            //--------------IN PROGRESS ---------------
            boolean reset = this.gamepad1.y;
            //---------------Coming Soon!--------------

            if (reset){
                robot.claw2.setPosition(0.9);

            }

            if(grip1 > 0.5) {
                robot.claw1.setPosition(0.45);
            }
            if(grip2>0.5) {
                robot.claw1.setPosition (0.3);
            }if (arm_down) {
                robot.arm.setTargetPosition(600);
                robot.arm.setPower(0.6);
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
            if(rotate1 ){
                robot.claw2.setPosition(0.35);
            }
            if(rotate2){
                robot.claw2.setPosition(0.9);
            }


        }



    }

}