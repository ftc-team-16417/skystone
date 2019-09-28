package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="test")
public class firstTeleop extends LinearOpMode {
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
        UpMotor = hardwareMap.get(DcMotor.class, "elevator");
        SidewaysMotor = hardwareMap.get(DcMotor.class, "scoop");
        Arm = hardwareMap.get(Servo.class, "arm");
        Button = hardwareMap.get(TouchSensor.class, "ts");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Left motor", LeftMotor.getCurrentPosition());
            telemetry.update();
            telemetry.addData("Right motor", RightMotor.getCurrentPosition());
            telemetry.update();

            double right = -gamepad1.right_stick_y;
            double left = -gamepad1.left_stick_y;
            LeftMotor.setPower(left);
            RightMotor.setPower(right);
            boolean up = gamepad1.y;
            boolean down = gamepad1.a;
            boolean SideRight = gamepad1.b;
            boolean SideLeft = gamepad1.x;
            boolean ArmUp = gamepad1.dpad_up;
            boolean ArmDown = gamepad1.dpad_down;


            if (up == true) {
                UpMotor.setPower(1);
            } else if (down == true) {
                UpMotor.setPower(-1);
            } else {
                UpMotor.setPower(0);
            }
            if (SideRight == true) {
                SidewaysMotor.setPower(-1);
            } else if (SideLeft == true) {
                SidewaysMotor.setPower(1);
            } else {
                SidewaysMotor.setPower(0);
            }
            if (ArmUp == true) {
                Arm.setPosition(1);
            } else if (ArmDown == true) {
                Arm.setPosition(0);
            }
            if (Button.isPressed() == true) {
                Arm.setPosition(0.5);
            }

        }

        while (LeftMotor.getCurrentPosition() < 1000 && RightMotor.getCurrentPosition() < 1000) {
            telemetry.addData("Left motor", LeftMotor.getCurrentPosition());
            telemetry.update();
            telemetry.addData("Right motor", RightMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
