package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name="encoder test")
public class drive_auto extends LinearOpMode {
    double WHEEL_SLIP_FACTOR = 1.05;
    @Override
    public void runOpMode() {
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap, telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot,action);
        waitForStart();
        drive_forward(90,robot);
        grab(robot);
        drive_forward(-50,robot);
        imu.rotate(90,0.5);
        drive_forward(200,robot);
        drop(robot);
        drive_forward(-139,robot);
        imu.rotate(7,0.5);
        drive_forward(50,robot);
        grab(robot);
        drive_forward(-50,robot);
        imu.rotate(90,0.5);
        drive_forward(150,robot);
        drop(robot);
        drive_forward(-100,robot);
       while (opModeIsActive()){

           telemetry.addData("imu",imu.getAngle());
           telemetry.update();
       }

    }
    void drive_forward(float distance,robot_hardware robot) {

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = cm_to_ticks(distance);
        robot.lf_drive.setTargetPosition(target);
        robot.rf_drive.setTargetPosition(target);
        robot.lr_drive.setTargetPosition(target);
        robot.rr_drive.setTargetPosition(target);
        robot.lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf_drive.setPower(1);
        robot.rf_drive.setPower(-1);
        robot.lr_drive.setPower(1);
        robot.rr_drive.setPower(-1);

        while (robot.lf_drive.isBusy()) ;
        robot.lf_drive.setPower(0);
        robot.rf_drive.setPower(0);
        robot.lr_drive.setPower(0);
        robot.rr_drive.setPower(0);
        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void grab(robot_hardware robot){
        robot.claw1.setPosition(0.3);
        robot.arm.setTargetPosition(1500);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(-1);
        while(robot.arm.isBusy());
        robot.arm.setPower(0);
        robot.claw1.setPosition(0.1);
        sleep(1000);
        robot.arm.setTargetPosition(100);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(-1);
        while(robot.arm.isBusy());
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drop(robot_hardware robot){
        robot.arm.setTargetPosition(1000);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(-1);
        while(robot.arm.isBusy());
        robot.arm.setPower(0);
        robot.claw1.setPosition(0.3);
        robot.arm.setTargetPosition(100);
        robot.arm.setPower(-1);
        while(robot.arm.isBusy());
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int cm_to_ticks(float distance){
        return (int)Math.ceil(distance*45.137/ Math.sqrt(2)*WHEEL_SLIP_FACTOR);
    }


}
