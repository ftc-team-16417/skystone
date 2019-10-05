package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class sample_class extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
        robot.init_imu(telemetry);
        action_lib action = new action_lib(robot);
        waitForStart();
        //code goes here

    }
}
