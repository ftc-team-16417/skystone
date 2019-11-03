package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Drive Imu")
public class Straight extends LinearOpMode
{
    /*
    DcMotor                 left, rightMotor;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .70, correction;
    double lasterror;
    double ticks = 4/(Math.PI*10.2);
    double last_error= 0;

    // called when init button is  pressed.

     */
    double ticks = 4/(Math.PI*10.2);
    @Override
    public void runOpMode() throws InterruptedException {
      /*leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        robot_hardware robot = new robot_hardware(hardwareMap, telemetry);
        action_lib action = new action_lib(robot);

        imu_lib imu = new imu_lib(robot, action);


        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        DrivePDcm(100,0.9,robot,imu);


    }



    public void DrivePDcm (double cm,double powerVoid,robot_hardware robot,imu_lib imu)
        {

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(robot.rf_drive.getCurrentPosition() < cm*ticks) {

            double correction = imu.getProportionalTerm(0,0.01,0.05);//get correction value
            robot.lf_drive.setPower(powerVoid - correction);
            robot.lr_drive.setPower(powerVoid + correction);
            robot.rr_drive.setPower(powerVoid + correction);
            robot.rf_drive.setPower(powerVoid + correction);
        }
    }

}