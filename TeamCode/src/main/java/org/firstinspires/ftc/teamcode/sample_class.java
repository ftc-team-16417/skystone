package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "something")
public class sample_class extends LinearOpMode {
    robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
    action_lib action = new action_lib(robot);
    imu_lib imu = new imu_lib(robot,action);
    @Override
    public void runOpMode(){
        waitForStart();

        //code goes here
        imu.turn(90,50);

    }
}
