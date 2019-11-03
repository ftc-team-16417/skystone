package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends LinearOpMode {

    public static BNO055IMU imu;


    @Override
    public void runOpMode(){
        double speed;
        speed = 0;
        robot_hardware robot  = new robot_hardware(hardwareMap,telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot,action);
        waitForStart();

       while(true){

           robot.rr_drive.setPower(speed);
           robot.lr_drive.setPower(speed);
           robot.rf_drive.setPower(-speed);
           robot.lf_drive.setPower(-speed);

       }


    }


}
