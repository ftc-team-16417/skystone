package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Lib extends Define {
    public void Lib(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();
    }

    public int ticks(double distance){
        int tick;
        tick = (int) Math.ceil(distance/(Math.PI*10.16)*1440);
        return tick;
    }

    public double correct(double original_angle){
        double correction, gain = -.050;

        double angle = getAngle();
        double error  = angle-original_angle;
        correction = error*gain;

        return correction;
    }

    public double getAngle() {

        Orientation newangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = newangle.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        //deltaAngle = (abs(deltaAngle)/deltaAngle)*(abs(deltaAngle) > 180)? ((abs(deltaAngle) % 180) + abs(deltaAngle)) : abs(deltaAngle);

        globalAngle += deltaAngle;

        lastAngles = newangle;

        return globalAngle;


    }

    public void goStraight(double t, double angle, int directionCheck){// 1 for
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(directionCheck == 1) {
            while (leftMotor.getCurrentPosition() < ticks(t) || rightMotor.getCurrentPosition() < ticks(t)) {
                double correction = correct(angle);
                leftMotor.setPower(0.5 - correction);
                rightMotor.setPower(0.5 + correction);
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }else{

        }
    }

    public void turn(int degrees, double power) {

        double leftPower=0, rightPower=0;

        if (degrees < 0) {
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {
            leftPower = -power;
            rightPower = power;

        }
        if(degrees==0){
            if(getAngle()<0){
                leftPower = -power;
                rightPower = power;
            }
            else{
                leftPower = power;
                rightPower = -power;
            }
        }

        lf_drive.setPower(leftPower);
        lr_drive.setPower(leftPower);

        rf_drive.setPower(rightPower);
        rr_drive.setPower(rightPower);

        if (degrees >= 0) {
            while (getAngle() < degrees);
        }
        else if (degrees < 0) {

            while (getAngle() > degrees);
        }

        lf_drive.setPower(0);
        lr_drive.setPower(0);

        rf_drive.setPower(0);
        rr_drive.setPower(0);
    }


}
