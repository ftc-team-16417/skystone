package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm")
public class ArmGrab extends LinearOpMode {
    robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
    public void drive(int target,double speed){
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setTargetPosition(500);

        robot.arm.setPower(0.3);
        while(robot.arm.isBusy()){
            idle();
        }
        robot.arm.setPower(0);


}
    @Override
    public void runOpMode(){
        //robot.init_imu(telemetry);
        action_lib action = new action_lib(robot);
        waitForStart();
            int t = 2000;
        while (opModeIsActive()) {
            telemetry.addData("encoder",robot.arm.getCurrentPosition());
            telemetry.update();
        }
        }

    }

