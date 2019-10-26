package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="hamza")
public class sample_class extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware(hardwareMap,telemetry);
        action_lib action = new action_lib(robot);
        waitForStart();
       //code goes here
        while (true){
            double move = this.gamepad1.left_stick_x;
            robot.intake1.setPower(move);
            robot.intake2.setPower(-move);
        }
    }
}
