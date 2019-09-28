package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="holo")
public class Holonomic extends LinearOpMode{
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    @Override
    public void runOpMode(){
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        waitForStart();
        while(opModeIsActive()){
            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double cwRotate = gamepad1.right_stick_x;
            fl.setPower(forward-sideways+cwRotate);
            bl.setPower(forward+sideways+cwRotate);
            fr.setPower(-forward-sideways+cwRotate);
            br.setPower(-forward+sideways+cwRotate);

        }
    }
}
