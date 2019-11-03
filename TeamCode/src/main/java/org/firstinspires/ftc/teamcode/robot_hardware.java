package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class robot_hardware extends Drive {
    public static DcMotor lf_drive,lr_drive,rr_drive,rf_drive,arm,intake_left,intake_right;
    public static Servo claw1,claw2,grip;
    public static HardwareMap hw;
    public robot_hardware(HardwareMap hardware, Telemetry telemetry){
        hw = hardware;
        try {
            lf_drive = hw.get(DcMotor.class, "lf_drive");
            lr_drive = hw.get(DcMotor.class, "lr_drive");
            rf_drive = hw.get(DcMotor.class, "rf_drive");
            rr_drive = hw.get(DcMotor.class, "rr_drive");
            claw1 = hw.get(Servo.class, "claw1");
            claw2 = hw.get(Servo.class, "claw2");
            arm = hw.get(DcMotor.class, "arm");
            //grip = hw.get(Servo.class, "grip");
            imu = hw.get(BNO055IMU.class, "imu");
            intake_left = hw.get(DcMotor.class, "left_intake");
            intake_right = hw.get(DcMotor.class, "right_intake");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            imu.initialize(parameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!imu.isGyroCalibrated()) {
                //idle
            }
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e){
            telemetry.addLine("failed initialization, check your ids");
            telemetry.update();
        }
    }
}
