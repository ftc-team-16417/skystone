package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class action_lib {
    robot_hardware robot;
    public action_lib(robot_hardware robot){
        this.robot = robot;
    }

    public void drive(){
        this.robot.arm.setTargetPosition(700);
        this.robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot_hardware.arm.setPower(1);
        while(robot.arm.isBusy()){
            //idle
        }
        this.robot.arm.setPower(0);
        this.robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
