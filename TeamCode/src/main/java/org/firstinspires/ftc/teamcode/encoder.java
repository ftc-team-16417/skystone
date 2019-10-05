package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="encoder")
public class encoder extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;

    @Override
    public void runOpMode() {
        LeftMotor = hardwareMap.get(DcMotor.class, "left");
        RightMotor = hardwareMap.get(DcMotor.class, "right");
        initDrive();

        waitForStart();

        drive(ticks(10),0.9);

        displayInfo();
        sleep(10000);
    }
    public void drive(int target,double speed){
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LeftMotor.setTargetPosition(target);
        RightMotor.setTargetPosition(target);
        LeftMotor.setPower(speed);
        RightMotor.setPower(speed);
        while (LeftMotor.isBusy() || RightMotor.isBusy()) {
            idle();
        }

        stopDrive();
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    void stopDrive() {
        LeftMotor.setPower(0);
        RightMotor.setPower(0);


    }
    public int ticks(double distance){
        int tick;
        tick = (int) Math.ceil(distance/(Math.PI*10.16)*1440);
        return tick;
    }
    public void initDrive(){
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void displayInfo(){
        telemetry.addData("left encoder", LeftMotor.getCurrentPosition());
        telemetry.addData("right encdoer", RightMotor.getCurrentPosition());
        telemetry.update();
    }
}
