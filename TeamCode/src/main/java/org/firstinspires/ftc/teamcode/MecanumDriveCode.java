package org.firstinspires.ftc.teamcode;


        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Mechanum")
public class MecanumDriveCode extends LinearOpMode {
    robot_hardware hardware = new robot_hardware();
    @Override
    public void runOpMode(){
        hardware.init(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            double strafe = -this.gamepad1.left_stick_x;
            double forward = -this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;
            double arm = this.gamepad1.left_trigger;

            hardware.rf_drive.setPower(-strafe+forward-turn);
            hardware.lf_drive.setPower(-strafe-forward-turn);
            hardware.rr_drive.setPower(strafe+forward-turn);
            hardware.lr_drive.setPower(strafe-forward-turn);

//arm
            double intakePower = this.gamepad1.left_trigger;
            boolean armPower1 = this.gamepad1.left_bumper;
            boolean armPower2 = this.gamepad1.right_bumper;
            hardware.right_intake.setPower(intakePower);
            hardware.left_intake.setPower(-intakePower);

            if(armPower1){
                hardware.arm.setPower(0.5);
            }else if(armPower2) {
                hardware.arm.setPower(-0.5);
            }



        }



    }

}