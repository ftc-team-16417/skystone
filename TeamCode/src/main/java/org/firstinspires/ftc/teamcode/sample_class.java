package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="newTeleop")
public class sample_class extends LinearOpMode{
    robot_hardware robot;
    @Override
    public void runOpMode(){
        robot = new robot_hardware(hardwareMap,telemetry);
        robot.init_imu(telemetry);
        action_lib action = new action_lib(robot);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        //code goes here
        while(opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double cwRotate = gamepad1.right_stick_x;
            boolean armUp = gamepad1.left_bumper;
            boolean armDown = gamepad1.right_bumper;
            boolean closeClaw = gamepad1.dpad_down;
            boolean openClaw = gamepad1.dpad_up;
            telemetry.addData("forward",forward);
            telemetry.addData("sideways",sideways);
            telemetry.addData("cwRotate", cwRotate);
            telemetry.update();
            telemetry.addData("arm", robot.arm.getCurrentPosition());
            telemetry.update();
            robot.lf_drive.setPower(forward-sideways+cwRotate);
            robot.lr_drive.setPower(forward+sideways+cwRotate);
            robot.rf_drive.setPower(-forward-sideways+cwRotate);
            robot.rr_drive.setPower(-forward+sideways+cwRotate);

            if(closeClaw == true){
                robot.claw1.setPosition(0);
            }
            else if(openClaw == true){
                robot.claw1.setPosition(1);
            }

            if(armUp){
                drive();

            }
            else if(armDown){
                while(robot.arm.getCurrentPosition() > 0){
                    robot.arm.setPower(-1);
                }
                robot.arm.setPower(0);
            }
            else{
                robot.arm.setPower(0);
            }
        }

    }
    public void drive(){
        robot_hardware.arm.setTargetPosition(700);
        robot_hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot_hardware.arm.setPower(1);
        while(robot.arm.isBusy()){
            idle();
        }
        robot_hardware.arm.setPower(0);
        robot_hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}