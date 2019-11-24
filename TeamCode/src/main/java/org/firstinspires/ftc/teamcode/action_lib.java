package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class action_lib {

    robot_hardware robot;
    action_lib(robot_hardware robot){
        this.robot = robot;
    }

    //your functions go here
    void run_drive(double lf,double rf,double rr,double lr){
        this.robot.lf_drive.setPower(lf);
        this.robot.lr_drive.setPower(lr);
        this.robot.rr_drive.setPower(rr);
        this.robot.rf_drive.setPower(rf);
    }
    void stop_drive(){
        this.robot.lf_drive.setPower(0);
        this.robot.lr_drive.setPower(0);
        this.robot.rr_drive.setPower(0);
        this.robot.rf_drive.setPower(0);
    }
    void pickUp() {
        this.robot.claw1.setPosition(0.35);
        for(int x = 0; x < 100000000; x++){
            //idle
        }
        this.robot.arm.setTargetPosition(1500);
        this.robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        this.robot.arm.setPower(1);
        while (robot.arm.isBusy()) {
            //idle
        }
        this.robot.claw1.setPosition(0);
        while(this.robot.claw1.getPosition() > 0){
            //idle
        }
        this.robot.arm.setPower(0);
        this.robot.arm.setTargetPosition(0);

        this.robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.arm.setPower(-1);
        while(robot.arm.isBusy()){
            //idle
        }
        this.robot.arm.setPower(0);
        this.robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void AutoMove(int target, Telemetry telemetry, imu_lib imu){
        this.robot.lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.robot.lr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.robot.rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.robot.rr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("lf_drive", robot.lf_drive.getCurrentPosition());
        telemetry.addData("lr_drive", robot.lf_drive.getCurrentPosition());
        telemetry.addData("rf_drive", robot.lf_drive.getCurrentPosition());
        telemetry.addData("rr_drive", robot.lf_drive.getCurrentPosition());
        telemetry.update();
        robot.lf_drive.setTargetPosition(-target);
        robot.lr_drive.setTargetPosition(-target);
        robot.rf_drive.setTargetPosition(target);
        robot.rr_drive.setTargetPosition(target);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.lf_drive.setPower(1);
        robot.lr_drive.setPower(1);
        robot.rf_drive.setPower(1);
        robot.rr_drive.setPower(1);

        while(robot.lf_drive.isBusy()){
            //idle
        }
        robot.lf_drive.setPower(0);
        robot.lr_drive.setPower(0);
        robot.rf_drive.setPower(0);
        robot.rr_drive.setPower(0);


    }



}
