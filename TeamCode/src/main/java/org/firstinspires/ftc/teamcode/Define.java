package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Define {
    public static DcMotor lf_drive,lr_drive,rr_drive,rf_drive,arm;
    public static Servo claw1,claw2,grip;
    public static BNO055IMU imu;
    public Define(HardwareMap hw){
        try {
            lf_drive = hw.get(DcMotor.class, "lf_drive");
            lr_drive = hw.get(DcMotor.class, "lr_drive");
            rf_drive = hw.get(DcMotor.class, "rf_drive");
            rr_drive = hw.get(DcMotor.class, "rr_drive");
            claw1 = hw.get(Servo.class, "claw1");
            claw2 = hw.get(Servo.class, "claw2");
            arm = hw.get(DcMotor.class, "arm");
            grip = hw.get(Servo.class, "grip");

        }
        catch (Exception e){

        }
    }
    public void initIMU(Telemetry telemetry){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!imu.isGyroCalibrated()) {
            //maybe idle later
        }

    }


}
