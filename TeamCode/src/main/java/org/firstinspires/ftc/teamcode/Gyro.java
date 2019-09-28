package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(name="gyro")
public class Gyro extends LinearOpMode {
    BNO055IMU imu;
    DcMotor RightMotor;
    DcMotor LeftMotor;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .70, correction;

    @Override
    public void runOpMode(){
        LeftMotor = hardwareMap. dcMotor.get("left");
        RightMotor = hardwareMap.dcMotor.get("right");
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        LeftMotor.setDirection(DcMotor.Direction.REVERSE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        telemetry.addData("mode","calibrating...");
        telemetry.update();
        sleep(50);
        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        telemetry.addData("mode", "waiting for status");
        //telemetry.addData("imu calib status", imu,getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
        int MoveAngle = 360;
        move(MoveAngle);



    }
    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double DeltaAngle = angles.firstAngle-lastAngles.firstAngle;
        if(DeltaAngle < -180){
            DeltaAngle = DeltaAngle + 360;
        }
        else if(DeltaAngle > 180){
            DeltaAngle = DeltaAngle - 360;
        }
        globalAngle+= DeltaAngle;

        lastAngles = angles;
        return globalAngle;
    }
    public void move(int MoveAngle){
        if(MoveAngle > getAngle()){
            LeftMotor.setPower(-0.5);
            RightMotor.setPower(0.5);
            while(true){
                telemetry.addData("angle ",globalAngle);
                telemetry.update();
                if(getAngle() >= MoveAngle){
                    LeftMotor.setPower(0);
                    RightMotor.setPower(0);
                    break;
                }

            }
        }
        if(MoveAngle < getAngle()){
            LeftMotor.setPower(0.5);
            RightMotor.setPower(-0.5);
            while(true){
                telemetry.addData("angle ",globalAngle);
                telemetry.update();
                if(getAngle() <= MoveAngle){
                    LeftMotor.setPower(0);
                    RightMotor.setPower(0);
                    break;
                }

            }
        }

        while(true){
            telemetry.addData("angle ",globalAngle);
            telemetry.update();
            if(getAngle() >= 175 || getAngle() <= -175){
                LeftMotor.setPower(0);
                RightMotor.setPower(0);
                break;
            }

        }
    }
}
