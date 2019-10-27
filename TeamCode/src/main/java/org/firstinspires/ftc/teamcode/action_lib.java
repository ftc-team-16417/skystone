package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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



}
