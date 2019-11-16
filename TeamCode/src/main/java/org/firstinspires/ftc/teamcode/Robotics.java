package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Teleop")
public class Robotics extends LinearOpMode {

    DcMotor LeftMotor;
    DcMotor RightMotor;
    DcMotor elevator1;
    DcMotor scoop;
    Servo servo;
    TouchSensor sensor;

    @Override
    public void runOpMode() {
        LeftMotor = hardwareMap.get(DcMotor.class, "left");
        RightMotor = hardwareMap.get(DcMotor.class, "right");
        scoop = hardwareMap.get(DcMotor.class, "scoop");
        elevator1 = hardwareMap.get(DcMotor.class, "elevator");
        servo = hardwareMap.get(Servo.class, "arm");
        sensor = hardwareMap.get(TouchSensor.class, "ts");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            //joystick
            double left = -this.gamepad1.left_stick_y;
            LeftMotor.setPower(left);
            double right = -this.gamepad1.right_stick_y;
            RightMotor.setPower(right);
            double motor3 = this.gamepad1.right_trigger;
            double motor4 = this.gamepad1.left_trigger;
            elevator1.setPower(motor3 - motor4);



                if (this.gamepad1.right_bumper) {
                    scoop.setPower(1);
                } else if (this.gamepad1.left_bumper) {
                    scoop.setPower(-1);
                } else {
                    scoop.setPower(0);
                }
                if (this.gamepad1.a) {
                    servo.setPosition(0);
                } else if (this.gamepad1.x) {
                    servo.setPosition(0.25);
                } else if (this.gamepad1.y) {
                    servo.setPosition((0.5));
                } else if (this.gamepad1.b) {
                    servo.setPosition(1);
                }

                telemetry.addData("left joystick", this.gamepad1.left_stick_y);
                telemetry.addData("right joystick", this.gamepad1.right_stick_y);
                telemetry.update();


        }

    }
}