package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="jeffisgay")
public class Autonomous_omni extends LinearOpMode {

    @Override
    public void runOpMode() {
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap, telemetry);
        action_lib action = new action_lib(robot);
        imu_lib imu = new imu_lib(robot, action);
        waitForStart();
        while (opModeIsActive()) {
            boolean move = this.gamepad1.y;
            if (move == true) {
                action.AutoMove(cm_to_ticks(100), telemetry, imu);
            }

        }

    }

    int cm_to_ticks(double distance) {
        double rotation = 1.4*(distance / (Math.PI * 12));
        return (int) Math.floor(rotation * 383.6);
    }

}


