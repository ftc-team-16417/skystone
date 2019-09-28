package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name="encoders")
public class Autonomous extends  LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;
    DcMotor UpMotor;
    DcMotor SidewaysMotor;
    Servo Arm;
    TouchSensor Button;
    @Override
    public void runOpMode() {
        LeftMotor = hardwareMap.get(DcMotor.class, "left");
        RightMotor = hardwareMap.get(DcMotor.class, "right");
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        UpMotor = hardwareMap.get(DcMotor.class, "elevator");
        SidewaysMotor = hardwareMap.get(DcMotor.class, "scoop");
        Arm = hardwareMap.get(Servo.class, "arm");
        Button = hardwareMap.get(TouchSensor.class, "ts");
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        Run(20, 1);

        sleep(10000);
    }
    public static int Converter(int distance){
        int Diameter = 10;

        int ticks =(int)Math.floor( distance/(Diameter*Math.PI)*1440);
        return ticks;
    }
    public void Run(int distance,double power){
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = Converter(distance);
        LeftMotor.setTargetPosition(ticks);
        RightMotor.setTargetPosition(ticks);
        LeftMotor.setPower(power);
        RightMotor.setPower(power);
        while (LeftMotor.isBusy() && RightMotor.isBusy()) {
            telemetry.addData("Left motor", LeftMotor.getCurrentPosition());
            telemetry.addData("Right motor", RightMotor.getCurrentPosition());
            telemetry.update();
        }
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }
}

