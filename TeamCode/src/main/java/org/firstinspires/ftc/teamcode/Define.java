package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .70, correction;

    @Override
    public void runOpMode() throws InterruptedException {
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        lr_drive = hardwareMap.get(DcMotor.class, "lr_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        rr_drive = hardwareMap.get(DcMotor.class, "rr_drive");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        arm =  hardwareMap.get(DcMotor.class, "arm");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf_drive.setDirection(DcMotor.Direction.REVERSE);
        lr_drive.setDirection(DcMotor.Direction.REVERSE);
        
    }


}
