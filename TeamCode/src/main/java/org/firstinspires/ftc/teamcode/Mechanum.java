package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Define;
@TeleOp(name = "Mechanum")
public class Mechanum extends LinearOpMode {

    @Override
    public void runOpMode(){


        Define robot  = new Define(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;
            double arm = this.gamepad1.left_trigger;

            robot.lf_drive.setPower(-strafe+forward-turn);
            robot.rf_drive.setPower(-strafe-forward-turn);
            robot.lr_drive.setPower(strafe+forward-turn);
            robot.rr_drive.setPower(strafe-forward-turn);

//arm
                double grip1 = this.gamepad1.left_trigger;
                boolean grip2 = this.gamepad1.left_bumper;
                if(grip1 > 0.5) {
                    robot.grip.setPosition(0.25);
                }
                if(grip2) {
                    robot.grip.setPosition (-0.25);
                }



            }



        }

    }