package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="testNotAnAuton")
public class Auton1 extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot, action);
        while(robot.arm.getCurrentPosition()>10) robot.arm.setPower(-1);
        imu.resetAngle();
        waitForStart();
        //code goes here

        imu.goStraightIMU(1, 0,100,0.5,robot);
        //imu.rotate(90,0.5);

    }
}
