package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class action_lib {

    static robot_hardware robot;
    public action_lib(robot_hardware robot){
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
    void yeeeeet(){
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setTargetPosition(800);
        robot.arm.setPower(1);
        for(int i=0;i<50000;i++);//wait
        robot.claw1.setPosition(0.35);
        while (robot.arm.isBusy()) ;
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void pick(){
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.claw1.setPosition(0.35);
        robot.arm.setTargetPosition(1200);
        robot.arm.setPower(0.7);
        while(robot.arm.isBusy());
        robot.arm.setPower(0);
        robot.claw1.setPosition(0.1);
        for(int i=0;i<50000;i++);//wait
        robot.arm.setTargetPosition(490);
        robot.arm.setPower(0.7);
        while(robot.arm.isBusy());
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
