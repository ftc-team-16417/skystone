package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Drive Imu")
public class IMU extends LinearOpMode {
    DcMotor leftMotor, rightMotor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .70, correction;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();



            move(100,0);
            sleep(500);
            rotate(90,1);
            sleep(500);
            move(10,90);
            sleep(500);
            rotate(-90,1);
            move(100,-90);

    }

    public int ticks(double distance){
        int tick;
        tick = (int) Math.ceil(distance/(Math.PI*10.16)*1440);
        return tick;
    }


    public void move(double t, double angle){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(leftMotor.getCurrentPosition()<ticks(t) || rightMotor.getCurrentPosition()<ticks(t)){
            double correction = correct(angle);
            leftMotor.setPower(0.5- correction);
            rightMotor.setPower(0.5+correction);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
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


    public void rotate(int degrees, double power) {

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

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (degrees >= 0) {
            while (getAngle() < degrees);
        }
        else if (degrees < 0) {

            while (getAngle() > degrees);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }




}