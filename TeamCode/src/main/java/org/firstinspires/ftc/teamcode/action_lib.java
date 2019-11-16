package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class action_lib{
    double OMNI_RATIO =1;
    robot_hardware this_robot;
    public action_lib(robot_hardware robot){
        this_robot = robot;
    }
    public void drive(int target,double speed,boolean strafe){
        this_robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this_robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this_robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this_robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this_robot.rr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this_robot.lr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this_robot.rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this_robot.lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        target = (int)Math.floor(ticks(target)*OMNI_RATIO);
        if(!strafe) {
            this_robot.rr_drive.setTargetPosition(-target);
            this_robot.lr_drive.setTargetPosition(target);
            this_robot.rf_drive.setTargetPosition(-target);
            this_robot.lf_drive.setTargetPosition(target);


            this_robot.rr_drive.setPower(speed);
            this_robot.lr_drive.setPower(-speed);
            this_robot.rf_drive.setPower(speed);
            this_robot.lf_drive.setPower(-speed);
        }else {
            this_robot.rr_drive.setTargetPosition(target);
            this_robot.lr_drive.setTargetPosition(target);
            this_robot.rf_drive.setTargetPosition(-target);
            this_robot.lf_drive.setTargetPosition(-target);


            this_robot.rr_drive.setPower(speed);
            this_robot.lr_drive.setPower(speed);
            this_robot.rf_drive.setPower(-speed);
            this_robot.lf_drive.setPower(-speed);


        }
        while (this_robot.rr_drive.isBusy() || this_robot.lr_drive.isBusy() || this_robot.rf_drive.isBusy() || this_robot.lf_drive.isBusy()) {

        }

        stopDrive();
        this_robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this_robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this_robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this_robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    void stopDrive() {
        this_robot.rr_drive.setPower(0);
        this_robot.lr_drive.setPower(0);
        this_robot.rf_drive.setPower(0);
        this_robot.lf_drive.setPower(0);


    }
    public int ticks(double distance){
        int tick;
        tick = (int) Math.ceil(distance/(Math.PI*10.16)*1440);
        return tick;
    }
    public void initDrive(){
        this_robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this_robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this_robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this_robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this_robot.rr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this_robot.lr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this_robot.rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this_robot.lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void run_drive(double lf,double rf,double rr,double lr){
        this_robot.lf_drive.setPower(lf);
        this_robot.lr_drive.setPower(lr);
        this_robot.rr_drive.setPower(rr);
        this_robot.rf_drive.setPower(rf);
    }
    void stop_drive(){
        this_robot.lf_drive.setPower(0);
        this_robot.lr_drive.setPower(0);
        this_robot.rr_drive.setPower(0);
        this_robot.rf_drive.setPower(0);
    }




}
