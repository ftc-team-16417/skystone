package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.Lib;

public class new_teleop extends LinearOpMode {
    public static Define robot;
    @Override
    public void runOpMode(){
        robot = new Define(hardwareMap);

    }
}
