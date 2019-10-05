package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private static DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2;

    public Drive(HardwareMap robot){
        leftDrive1 = robot.get(DcMotor.class, "lf");
        leftDrive2 = robot.get(DcMotor.class, "lr");
        rightDrive1 = robot.get(DcMotor.class, "rf");
        rightDrive2 = robot.get(DcMotor.class, "rr");
    }
    public static void doDrive(int lin, int rot){
        leftDrive1.setPower(lin - rot);
        leftDrive2.setPower(lin - rot);
        rightDrive1.setPower(lin + rot);
        rightDrive2.setPower(lin + rot);
    }
    public static void driveToPos(int pos){
        while(leftDrive1.getCurrentPosition() < pos) {
            int error = pos - leftDrive1.getCurrentPosition();
            leftDrive1.setPower(error * 1.5);
        }
    }

}
