package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="taco nacho")
public class TacoNacho extends LinearOpMode {
    DcMotor arm;
    DcMotor bl;
    DcMotor br;
    DcMotor fl;
    DcMotor fr;
    Servo claw1;
    Servo claw2;
    @Override
    public void runOpMode(){
        arm = hardwareMap.get(DcMotor.class,"arm");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br= hardwareMap.get(DcMotor.class,"br");
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();
            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double cwRotate = gamepad1.right_stick_x;
            boolean servoUp = gamepad1.dpad_up;
            boolean servoDown = gamepad1.dpad_down;
            boolean armUp = gamepad1.left_bumper;
            boolean armDown = gamepad1.right_bumper;
            fl.setPower(forward-sideways+cwRotate);
            bl.setPower(forward+sideways+cwRotate);
            fr.setPower(-forward-sideways+cwRotate);
            br.setPower(-forward+sideways+cwRotate);

            if(servoDown == true){
                claw1.setPosition(0);
                claw2.setPosition(1);
            }
            if(servoUp == true){
                claw1.setPosition(1);
                claw2.setPosition(0);
            }
            if(armDown == true){
                arm.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy()){

                }
                arm.setPower(0);
            }
            if (armUp == true) {
               arm.setTargetPosition(960);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
                while(arm.isBusy()){

                }
                arm.setPower(0);
            }
        }
    }
}
