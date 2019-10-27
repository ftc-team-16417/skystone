package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="test")
public class sample_class extends LinearOpMode {
    robot_hardware robot = new robot_hardware();
    action_lib action = null;
    imu_lib imu = null;

    @Override
    public void runOpMode() throws RuntimeException{
        robot.init(hardwareMap, telemetry);
        action = new action_lib(robot);
        imu = new imu_lib(robot,action);
        waitForStart();
        double test1 = imu.getAngle();

        while (opModeIsActive()){
            double forward = this.gamepad1.left_stick_x;
            double strafe = this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;
            boolean auto = this.gamepad1.a;
            action.run_drive(forward-strafe-turn,forward+strafe+turn,
                    -forward+strafe+turn,forward+strafe-turn);


            if(auto == true){
                action.pickUp();
            }
        }
    }
}
