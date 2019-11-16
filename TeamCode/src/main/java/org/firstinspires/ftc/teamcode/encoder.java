package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="encoder")
public class encoder extends LinearOpMode {
    robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
    @Override
    public void runOpMode() {

        initDrive();

        waitForStart();

        drive(ticks(10),0.9);

        displayInfo();
        sleep(10000);
    }
    public void drive(int target,double speed){
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.lf_drive.setTargetPosition(target);
        robot.lr_drive.setTargetPosition(target);
        robot.rr_drive.setTargetPosition(target);
        robot.rf_drive.setTargetPosition(target);
        robot.lf_drive.setPower(speed);
        robot.lr_drive.setPower(speed);
        robot.rr_drive.setPower(speed);
        robot.rf_drive.setPower(speed);


        while (robot.lf_drive.isBusy() || robot.lr_drive.isBusy() || robot.rr_drive.isBusy() || robot.rf_drive.isBusy()) {
            idle();
        }

        stopDrive();
        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    void stopDrive() {
        robot.lf_drive.setPower(0);
        robot.lr_drive.setPower(0);
        robot.rr_drive.setPower(0);
        robot.rf_drive.setPower(0);


    }
    public int ticks(double distance){
        int tick;
        tick = (int) Math.ceil(distance/(Math.PI*10.16)*1440);
        return tick;
    }
    public void initDrive(){
        robot.lf_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lr_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void displayInfo(){
        telemetry.addData("left front encoder", robot.lf_drive.getCurrentPosition());
        telemetry.addData("left rear encdoer", robot.lr_drive.getCurrentPosition());
        telemetry.addData("right rear enconder", robot.rr_drive.getCurrentPosition());
        telemetry.addData("right front encoder", robot.rf_drive.getCurrentPosition());
        telemetry.update();
    }
}
