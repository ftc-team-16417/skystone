package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;


@TeleOp
//@Disabled
public class MecanumResetArm extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    MecanumChassis robot   = new MecanumChassis();   // Use a Mecanum chassis's hardware

    double rotationServoPos = 0.5;
    double rotationServoPosStep = 0.02;
    int rotationDelayCnt = 0;

    int arm_tilt_Pos = 0;
    int arm_tilt_PosStep = 60;
    int arm_tilt_PosDelayCnt = 0;
    final int ARM_TILT_POS_LM = 6050;
    int arm_tilt_Pos_index = 0;
    int[] arm_tilt_Pos_Array = new int[]{0,351,2953,5586,6050};
    boolean arm_tilt_button_release = true;
    int arm_tilt_button_delay_cnt = 0;



    int arm_stretch_Pos = 0;
    int arm_stretch_PosStep = 50;
    int arm_stretch_PosDelayCnt = 0;
    final int ARM_STRETCH_POS_LM = -1400;

    boolean clawOpenFlag = true;
    int yButtonDelayCnt = 0;
    boolean yButtonReleaseFlag = true;

    boolean hookDown = false;
    int hookBtnDelayCnt = 0;
    boolean hookButtonreleaseFlag = true;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // set left motor to run to 0 encoder counts, it supposes at current reset position
        arm_tilt_Pos = 0;
        arm_stretch_Pos = 0;
        robot.arm_stretch.setTargetPosition(arm_stretch_Pos);
        robot.arm_tilt.setTargetPosition(arm_tilt_Pos);


        robot.arm_tilt.setPower(1);
        robot.arm_stretch.setPower(1);

        robot.claw_rotate.setPosition(rotationServoPos);
        robot.claw_open.setPosition(0.5);
        robot.arm_tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm_stretch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("encoder",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());
        telemetry.update();

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // According to RobotDrive.mecanumDrive_Cartesian in WPILib:
        //
        // LF =  x + y + rot    RF = -x + y - rot
        // LR = -x + y + rot    RR =  x + y - rot
        //
        // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
        // => (LF + RR) - (RF + LR) = 4x
        // => x = ((LF + RR) - (RF + LR))/4
        //
        // LF + RF + LR + RR = 4y
        // => y = (LF + RF + LR + RR)/4
        //
        // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
        // => (LF + LR) - (RF + RR) = 4rot
        // => rot = ((LF + LR) - (RF + RR))/4
        //
        double y = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;
        leftFrontPower = x + y + rot;
        rightFrontPower = -x + y - rot;
        leftRearPower = -x + y + rot;
        rightRearPower = x + y - rot;

        leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
        rightFrontPower   = Range.clip(rightFrontPower, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower, -1.0, 1.0) ;
        rightRearPower   = Range.clip(rightRearPower, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.


        // Send calculated power to wheels
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftRearDrive.setPower(leftRearPower);
        robot.rightRearDrive.setPower(rightRearPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //display joystick power
        //telemetry.addData("Motors", "leftF (%.2f), rightF (%.2f),leftR (%.2f), rightR (%.2f)", leftFrontPower, rightFrontPower,leftRearPower, rightRearPower);
        //display encoder reading
        telemetry.addData("encoder",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                //robot.rightFrontDrive.getCurrentPosition(),
                //robot.rightRearDrive.getCurrentPosition()
                robot.arm_tilt.getCurrentPosition(),
                robot.arm_stretch.getCurrentPosition());


        //for intake control here
        if (gamepad1.left_trigger > 0){
            // take in
            robot.intake_right.setPower(1);
            robot.intake_left.setPower(1);
        }else if(gamepad1.left_bumper ){
            //push out
            robot.intake_right.setPower(-1);
            robot.intake_left.setPower(-1);
        }else{
            robot.intake_right.setPower(0);
            robot.intake_left.setPower(0);
        }


        if (gamepad1.right_bumper){
            robot.arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm_tilt.setPower(0.5);
        }else if(gamepad1.right_trigger > 0){
                robot.arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm_tilt.setPower(-0.5);
        }
        else{

            robot.arm_tilt.setPower(0);

        }


        //arm stretch
        if (gamepad1.dpad_up){
            //for arm stretch out


        }else if(gamepad1.dpad_down){

        }
        else{




        }

        if (gamepad1.start){
            arm_stretch_Pos = 0;
            robot.arm_stretch.setTargetPosition(arm_stretch_Pos);
            robot.arm_stretch.setPower(1);
        }


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}