package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="testAiden")
public class sample_class extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot, action);

        robot.arm.setPower(0.3);
        try{
            wait(100);
        }catch(Exception e){}
        while(robot.arm.getCurrentPosition()<=100) {
            robot.arm.setPower(-0.05);
        }
        waitForStart();
        //code goes here
        imu.goStraightIMU(1, 0,20,0.5,robot);

    }
}
