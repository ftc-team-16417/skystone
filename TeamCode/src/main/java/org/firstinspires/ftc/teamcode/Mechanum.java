package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mechanum")
public class Mechanum extends LinearOpMode {

    DcMotor lf_drive;
    DcMotor lr_drive;
    DcMotor rf_drive;
    DcMotor rr_drive;
    @Override
    public void runOpMode(){
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        lr_drive = hardwareMap.get(DcMotor.class, "lr_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        rr_drive = hardwareMap.get(DcMotor.class, "rr_drive");

        waitForStart();

        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;
            lf_drive.setPower(-strafe+forward-turn);
            rf_drive.setPower(-strafe-forward-turn);
            lr_drive.setPower(strafe+forward-turn);
            rr_drive.setPower(strafe-forward-turn);



            }



        }

    }