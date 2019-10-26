
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "testUSB")
public class testUSB extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap,telemetry);
        telemetry.addLine("done calibration");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            robot.left_intake.setPower(1);
            sleep(1000);
            robot.left_intake.setPower(-1);
            sleep(1000);

        }
    }
}