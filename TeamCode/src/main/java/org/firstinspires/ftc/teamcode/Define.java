package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Define extends LinearOpMode {
    DcMotor lf_drive;
    DcMotor lr_drive;
    DcMotor rf_drive;
    DcMotor rr_drive;
    Servo claw1;
    Servo claw2;
    DcMotor arm;
    DcMotor leftMotor, rightMotor;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        lr_drive = hardwareMap.get(DcMotor.class, "lr_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        rr_drive = hardwareMap.get(DcMotor.class, "rr_drive");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        arm =  hardwareMap.get(DcMotor.class, "arm");
    }


}
