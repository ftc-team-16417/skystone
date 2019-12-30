package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import android.os.*;

import static java.lang.Math.abs;
import static java.lang.Math.signum;


@TeleOp
//@Disabled
public class MecanumTeleoperate_Angle extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    MecanumChassis robot   = new MecanumChassis();   // Use a Mecanum chassis's hardware


    double gearLevel = 0.5;
    final double CLAW_ROTATION_INI = 0.4;
    double rotationServoPos = CLAW_ROTATION_INI;
    double rotationServoPosStep = 0.2;
    int rotationDelayCnt = 0;

    int arm_tilt_Pos = 0;

    final int ARM_TILT_POS_LM = 3300;

    double arm_tilt_angle = 0;
    final double ARM_TILT_START = 77.0;               // reset position, need measure, down vertical is 0
    final double ARM_TILT_ANGLE_RES = 15.4778;     // 2786 * 2 / 360 = 14.477778
    final double ARM_TILT_MAX_POWER_1 = 0.6;              //need test, this power is for 90 degree to hold the arm
    final double ARM_TILT_MIN_POWER = 0.2;          // need test minimum move /hold the arm power
    double arm_tilt_power = 0;              // this one will be dynamic updated based on the arm tilt angle

    int arm_stretch_Pos = 0;

    final int ARM_STRETCH_POS_LM = -1360;
    final double ARM_STRETCH_RES = 1 / (ARM_STRETCH_POS_LM);
    double arm_stretch_len = 0;
    double arm_stretch_power = 0;
    final double ARM_STRETCH_MIN_POWER = 0.3;
    final double ARM_STRETCH_MAX_POWER = 1.0;
    final int ARM_PICK_READY_POS = 155;
    final int ARM_PICKUP_POS = 0;
    final int ARM_PICKUP_MIDDLE_POS1 = 200;
    final int ARM_PICKUP_MIDDLE_POS2 = 700;
    final int ARM_PICKUP_MIDDLE_POS3 = 1000;
    final int ARM_DROP_LEVEL1 = 3288;
    final int ARM_DROP_LEVEL2 = 2895;
    final int ARM_DROP_LEVEL3 = 2600;
    final int ARM_DROP_LEVEL4 = 2400;
    final int ARM_DROP_LEVEL5 = 2200;
    final int ARM_DROP_LEVEL6 = 2000;
    final int ARM_DROP_LEVEL7 = 1850;
    int arm_tilt_Pos_index = 0;
    int[] arm_tilt_Pos_Array = new int[]{ARM_PICKUP_POS,ARM_PICK_READY_POS,ARM_PICKUP_MIDDLE_POS1,ARM_PICKUP_MIDDLE_POS2,
                                        ARM_PICKUP_MIDDLE_POS3,ARM_DROP_LEVEL7,ARM_DROP_LEVEL6,ARM_DROP_LEVEL5,ARM_DROP_LEVEL4,
                                        ARM_DROP_LEVEL3,ARM_DROP_LEVEL2,ARM_DROP_LEVEL1};

    boolean armInCtrlTask = false;
    boolean armInCtrl = false;



    ArmPickUPThread armPickUPThread = null;
    Arm2PickUPReadyThread arm2PickUPReadyThread = null;
    Arm2Level7PosThread arm2Level7PosThread = null;

    final int STRETCH_MIDDLE_POS = - 400;
    final int STRETCH_RESET_POS = 0;
    boolean stretchPosCtrlFlag = false;
    boolean stretchInCtrlTask = false;
    int stretchTargetPos = 0;


    final double CLAW_OPEN_POS = 0.52;
    final double CLAW_CLOSE_POS =0.4;





    boolean clawOpenFlag = true;
    int bBtnDelayCnt = 0;
    boolean bBtnReleaseFlag = true;

    boolean xBtnFlag = false;
    int xBtnDelayCnt = 0;
    boolean xBtnReleaseFlag = true;

    boolean yBtnFlag = false;
    int yBtnDelayCnt = 0;
    boolean yBtnReleaseFlag = true;

    boolean aBtnFlag = false;
    int aBtnDelayCnt = 0;
    boolean aBtnReleaseFlag = true;


    final double CLAW_TILT_START = 0.86;
    final double CLAW_TILT_END = 0.12;
    final double CLAW_TILT_RATIO = (CLAW_TILT_START - CLAW_TILT_END) / (360 - 2 * ARM_TILT_START);
    double claw_tilt_cmd = CLAW_TILT_START;


    private double[] powerTable1 = {0.0,0.0, 0.0, 0.0,0.00,0.00,0.00,0.0,0.05,0.06,
                                    0.07,0.08, 0.09, 0.1,0.1,0.1,0.1,0.1,0.1,0.1,
                                    0.15,0.15, 0.15, 0.15,0.15,0.15,0.15,0.15,0.15,0.15,
                                    0.15,0.15, 0.15, 0.15,0.15,0.15,0.15,0.15,0.15,0.15,
                                    0.2,0.2, 0.2, 0.2,0.2,0.2,0.2,0.2,0.2,0.2,
                                    0.3,0.3, 0.3, 0.3,0.35,0.35,0.35,0.4,0.4,0.4,
                                    0.4,0.4, 0.45, 0.45,0.45,0.45,0.50,0.5,0.5,0.5,
                                    0.55,0.55, 0.55, 0.6,0.6,0.6,0.65,0.65,0.65,0.7,
                                    0.75,0.75, 0.8, 0.8,0.85,0.85,0.9,0.9,0.95,0.95,
                                    0.97,0.97, 0.98, 0.98,0.99,0.99,1.0,1.0,1.0,1.0};


    final double LEFT_HOOK1_INI = 0.48;
    final double LEFT_HOOK2_INI = 0.49;
    final double LEFT_HOOK_RANGE = 0.393;
    final double RIGHT_HOOK1_INI = 0.55;
    final double RIGHT_HOOK2_INI = 0.472;
    final double RIGHT_HOOK_RANGE = 0.403;
    boolean blnLeftHookDown = false;       // initial at up position
    boolean blnBackBtnRelease = true;
    int backBtnDelayCnt = 0;


    final double LEFT_CLAWU_INI = 0.55;
    final double LEFT_CLAWL_INI = 0.575;
    final double LEFT_CLAWU_PICK = LEFT_CLAWU_INI - 0.3 ;
    final double LEFT_CLAWL_PICK = LEFT_CLAWL_INI - 0.38;
    final double LEFT_CLAWU_HOLD = LEFT_CLAWU_INI + 0.1;
    final double LEFT_CLAWL_HOLD = LEFT_CLAWL_INI -  0.1;
    final double LEFT_CLAWU_RELEASE = LEFT_CLAWU_INI - 0.3;

    final double RIGHT_CLAWU_INI = 0.44;
    final double RIGHT_CLAWL_INI = 0.47;
    final double RIGHT_CLAWU_PICK = RIGHT_CLAWU_INI + 0.2 ;
    final double RIGHT_CLAWL_PICK = RIGHT_CLAWL_INI + 0.34;
    final double RIGHT_CLAWU_HOLD = RIGHT_CLAWU_INI - 0.05;
    final double RIGHT_CLAWL_HOLD = RIGHT_CLAWL_INI +  0.05;
    final double RIGHT_CLAWU_RELEASE = RIGHT_CLAWU_INI + 0.13;

    final double CAPSTONE_INI = 0.55;
    final double CAPSTONE_PUTDOWN = 0.135;
    boolean blnPutDownCapStone = false;
    PutDownCapStoneThread putDownCapStoneThread = null;

    boolean blnLeftClawInCtrl = false;


    boolean blnStartBtnRelease = true;
    int startBtnDelayCnt = 0;
    boolean blnHoldStone = false;


    public static final double NEW_P = 10.0;
    public static final double NEW_I = 0.0;
    public static final double NEW_D = 0.0;

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

        robot.claw_rotate.setPosition(rotationServoPos);
        robot.claw_open.setPosition(0.5);
        claw_tilt_cmd = CLAW_TILT_START;
        robot.claw_tilt.setPosition(claw_tilt_cmd);


        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_tilt.setPower(-0.1);
        try {
            Thread.sleep(600);
        } catch(InterruptedException e) {
            System.out.println("got interrupted!");
        }
        robot.arm_tilt.setPower(0);
        robot.arm_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try {
            Thread.sleep(100);
        } catch(InterruptedException e) {
            System.out.println("got interrupted!");
        }

        robot.arm_tilt.setPositionPIDFCoefficients(NEW_P);
        robot.arm_tilt.setTargetPositionTolerance(10);

        robot.arm_tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm_stretch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_tilt_angle = ARM_TILT_START;
        arm_tilt_Pos = ARM_PICK_READY_POS;
        robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
        robot.arm_stretch.setTargetPosition(arm_stretch_Pos);
        robot.arm_stretch.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm_tilt.setPower(0.3);



        robot.leftHook2.setPosition(LEFT_HOOK2_INI);
        robot.leftHook1.setPosition(LEFT_HOOK1_INI);
        robot.rightHook2.setPosition(RIGHT_HOOK2_INI);
        robot.rightHook1.setPosition(RIGHT_HOOK1_INI);
        blnLeftHookDown = false;

        robot.autoLeftClawL.setPosition(LEFT_CLAWL_INI);
        robot.autoLeftClawU.setPosition(LEFT_CLAWU_INI);
        robot.autoRightClawL.setPosition(RIGHT_CLAWL_INI);
        robot.autoRightClawU.setPosition(RIGHT_CLAWU_INI);

        robot.capStone.setPosition(CAPSTONE_INI);

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
        int index = Math.abs((int)(x * 100));
        index = (index >= 100? index = 99: index);
        x = Math.signum(x) * powerTable1[index];
        index = Math.abs((int)(y * 100));
        index = (index >= 100? index = 99: index);
        y = Math.signum(y) * powerTable1[index];
        index = Math.abs((int)(rot * 100));
        index = (index >= 100? index = 99: index);
        rot = Math.signum(rot) * powerTable1[index];

        leftFrontPower =  x + y + rot;
        rightFrontPower = -x + y - rot;
        leftRearPower = -x + y + rot;
        rightRearPower = x + y - rot;

        leftFrontPower    = Range.clip(leftFrontPower * gearLevel, -1.0, 1.0) ;
        rightFrontPower   = Range.clip(rightFrontPower * gearLevel, -1.0, 1.0) ;
        leftRearPower    = Range.clip(leftRearPower * gearLevel, -1.0, 1.0) ;
        rightRearPower   = Range.clip(rightRearPower * gearLevel, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.


        // Send calculated power to wheels
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftRearDrive.setPower(leftRearPower);
        robot.rightRearDrive.setPower(rightRearPower);

        updateArmPosition();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //display joystick power
        //telemetry.addData("Motors", "leftF (%.2f), rightF (%.2f),leftR (%.2f), rightR (%.2f)", leftFrontPower, rightFrontPower,leftRearPower, rightRearPower);
        //display encoder reading
        telemetry.addData("encoder",  "Starting at %7f :%7d:%7d:%7d",
                //robot.leftFrontDrive.getCurrentPosition(),
                gearLevel,
                //robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                //robot.rightRearDrive.getCurrentPosition()

                robot.arm_tilt.getCurrentPosition(),
                robot.arm_stretch.getCurrentPosition()
                );


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


        //for arm control here
        if (gamepad1.x){
            if (!xBtnFlag){
                if (!armInCtrlTask){
                    armPickUpCtrl();
                }
                xBtnFlag = true;
                xBtnDelayCnt = 0;
            }

        }else{
            xBtnDelayCnt ++;
            if (xBtnDelayCnt > 10){
                xBtnDelayCnt = 10;
                xBtnFlag = false;
            }

            if (!armInCtrlTask){
                if (armPickUPThread != null) {
                    armPickUPThread.interrupt();
                    armPickUPThread = null;
                }
            }
        }

        if (gamepad1.a){
            if (aBtnReleaseFlag){
                aBtnReleaseFlag = false;
                aBtnDelayCnt = 0;
                //move back to pick up read position
                arm2PickUpReadyPos();

            }

        }else {
            aBtnDelayCnt++;
            if (aBtnDelayCnt > 10) {
                aBtnDelayCnt = 10;
                aBtnReleaseFlag = true;
            }

            if (!armInCtrlTask){
                if (arm2PickUPReadyThread != null){
                    arm2PickUPReadyThread.interrupt();
                    arm2PickUPReadyThread = null;
                }
            }
        }


        if (gamepad1.y){
            if (yBtnReleaseFlag){
                yBtnReleaseFlag = false;

                arm2Level7Pos();
            }

        }else{
            yBtnDelayCnt ++;
            if (yBtnDelayCnt > 10){
                yBtnDelayCnt = 10;
                yBtnReleaseFlag = true;
            }

            if (!armInCtrlTask){
                if (arm2Level7PosThread != null){
                    arm2Level7PosThread.interrupt();
                    arm2Level7PosThread = null;
                }
            }



        }

        if (gamepad1.b){
            if (bBtnReleaseFlag){
                if (clawOpenFlag) {
                    robot.claw_open.setPosition(CLAW_CLOSE_POS);
                    clawOpenFlag = false;
                }
                else{
                    robot.claw_open.setPosition(CLAW_OPEN_POS);
                    clawOpenFlag = true;
                }
                bBtnDelayCnt = 0;
                bBtnReleaseFlag = false;
            }
        }
        else{
            bBtnDelayCnt ++;
            if (bBtnDelayCnt > 5){
                bBtnReleaseFlag = true;
                bBtnDelayCnt = 5;
            }
        }


        //servo pick1 for rotation, maybe we need change to continue mode
        if (gamepad1.dpad_right){
            rotationDelayCnt ++;
            if (rotationDelayCnt > 1) {
                rotationDelayCnt = 0;
                rotationServoPos += rotationServoPosStep;
                if (rotationServoPos > 1.0) rotationServoPos = 1.0;
                robot.claw_rotate.setPosition(rotationServoPos);
            }
        }
        else if (gamepad1.dpad_left){
            rotationDelayCnt ++;
            if (rotationDelayCnt > 1) {
                rotationDelayCnt = 0;
                rotationServoPos -= rotationServoPosStep;
                if (rotationServoPos < 0) rotationServoPos = 0;
                robot.claw_rotate.setPosition(rotationServoPos);
            }

        }
        else{
            rotationDelayCnt = 0;
        }


        if (gamepad1.back){
            if (blnBackBtnRelease){
                backBtnDelayCnt = 0;
                blnBackBtnRelease = false;
                if (!blnLeftHookDown){
                    robot.leftHook1.setPosition(LEFT_HOOK1_INI + LEFT_HOOK_RANGE);
                    robot.leftHook2.setPosition(LEFT_HOOK2_INI - LEFT_HOOK_RANGE);
                    robot.rightHook1.setPosition(RIGHT_HOOK1_INI - RIGHT_HOOK_RANGE);
                    robot.rightHook2.setPosition(RIGHT_HOOK2_INI + RIGHT_HOOK_RANGE);
                    blnLeftHookDown = true;
                }else {
                    blnLeftHookDown = false;
                    robot.leftHook1.setPosition(LEFT_HOOK1_INI);
                    robot.leftHook2.setPosition(LEFT_HOOK2_INI);
                    robot.rightHook1.setPosition(RIGHT_HOOK1_INI);
                    robot.rightHook2.setPosition(RIGHT_HOOK2_INI);
                }
            }
        }
        else{
            backBtnDelayCnt ++;
            if (backBtnDelayCnt > 10){
                backBtnDelayCnt = 10;
                blnBackBtnRelease = true;
            }
        }
// for gear change, not used now
        /*
        if (gamepad1.start){

            if (blnStartBtnRelease){
                blnStartBtnRelease = false;
                if (gearLevel == 0.3){
                    gearLevel = 1.0;
                }
                else{
                    gearLevel = 0.3;
                }
            }
            startBtnDelayCnt = 0;

        }
        else{
            startBtnDelayCnt ++;
            if (startBtnDelayCnt > 10){
                startBtnDelayCnt = 10;
                blnStartBtnRelease = true;
            }

        }
        */

 // for put down cap stone
        if (gamepad1.start){

            if (blnStartBtnRelease){
                blnStartBtnRelease = false;
                if (!blnPutDownCapStone){
                    putDownCapStone();
                }
            }
            startBtnDelayCnt = 0;

        }
        else{
            startBtnDelayCnt ++;
            if (startBtnDelayCnt > 10){
                startBtnDelayCnt = 10;
                blnStartBtnRelease = true;
            }

        }


        if ((!armInCtrlTask) && (!armInCtrl)) {

            if (gamepad1.right_bumper) {
                if (robot.arm_tilt.getCurrentPosition() < ARM_TILT_POS_LM){
                    if (robot.arm_tilt.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    robot.arm_tilt.setPower(arm_tilt_power);

                }

            } else if (gamepad1.right_trigger > 0) {
                if(robot.arm_tilt.getCurrentPosition() > 5){
                    if (robot.arm_tilt.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    robot.arm_tilt.setPower(-arm_tilt_power);
                }



            } else {
                if (robot.arm_tilt.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    robot.arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                robot.arm_tilt.setPower(0);


            }
        }

        //arm stretch
        if (!stretchInCtrlTask) {
            if (gamepad1.dpad_up) {
                if (robot.arm_stretch.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    robot.arm_stretch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (robot.arm_stretch.getCurrentPosition() > ARM_STRETCH_POS_LM){
                    robot.arm_stretch.setPower(-arm_stretch_power);
                }
                else{
                    robot.arm_stretch.setPower(0);
                }



            } else if (gamepad1.dpad_down) {

                if (robot.lowerSwitch.getState()) {
                    robot.arm_stretch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.arm_stretch.setPower(0.4);
                } else {

                    robot.arm_stretch.setPower(0);
                    robot.arm_stretch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

            } else {
                //hold current position
                robot.arm_stretch.setPower(0);

            }
        }


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void updateArmPosition(){
        //update the arm tilt power based on the arm tilt angle
        arm_tilt_angle = ARM_TILT_START + (robot.arm_tilt.getCurrentPosition() - 0) / ARM_TILT_ANGLE_RES;
        if (arm_tilt_angle <= 90){
            arm_tilt_power = ARM_TILT_MIN_POWER + (ARM_TILT_MAX_POWER_1 - ARM_TILT_MIN_POWER)/ (90 - ARM_TILT_START) * (arm_tilt_angle - ARM_TILT_START);
        }
        else if ((arm_tilt_angle > 90) && (arm_tilt_angle <= 180)){
            arm_tilt_power = ARM_TILT_MAX_POWER_1 -  (ARM_TILT_MAX_POWER_1 - ARM_TILT_MIN_POWER)/ (180 - 90) * (arm_tilt_angle - 90);
        }
        else if ((arm_tilt_angle > 180) && (arm_tilt_angle <= 270)){
            arm_tilt_power = ARM_TILT_MIN_POWER + (ARM_TILT_MAX_POWER_1 - ARM_TILT_MIN_POWER)/ (270 - 180) * (arm_tilt_angle - 180);
        }
        else if ((arm_tilt_angle > 270) && (arm_tilt_angle <= 270 + 90 - ARM_TILT_START)){
            arm_tilt_power = ARM_TILT_MAX_POWER_1 -  (ARM_TILT_MAX_POWER_1 - ARM_TILT_MIN_POWER)/ (90 - ARM_TILT_START ) * (arm_tilt_angle - 270);
        }
        else{
            arm_tilt_power = ARM_TILT_MIN_POWER;
        }


        //update the arm stretch position and power here

        //update the arm_tilt_power, stretch more need more power
        arm_tilt_power = arm_tilt_power ;
        //based on the arm_tilt angle to update the arm_stretch power here
        if ((arm_tilt_angle > 90) && (arm_tilt_angle <= 180)){
            arm_stretch_power = ARM_STRETCH_MIN_POWER + (ARM_STRETCH_MAX_POWER - ARM_STRETCH_MIN_POWER) / (180 - 90) * (arm_tilt_angle - 90);

        }
        else if ((arm_tilt_angle > 180) && (arm_tilt_angle <= 270)){
            arm_stretch_power = ARM_STRETCH_MIN_POWER + (ARM_STRETCH_MAX_POWER - ARM_STRETCH_MIN_POWER) / (270 - 180) * (270 - arm_tilt_angle);

        }
        else{
            arm_stretch_power = ARM_STRETCH_MIN_POWER;
        }


        //update the claw tilt based on the arm tilt angle
        claw_tilt_cmd = CLAW_TILT_START - CLAW_TILT_RATIO * (arm_tilt_angle - ARM_TILT_START);
        robot.claw_tilt.setPosition(claw_tilt_cmd);


    }


    // arm control thread 1
    private void armPickUpCtrl()
    {
        if (arm2Level7PosThread != null){
            arm2Level7PosThread.interrupt();
            arm2Level7PosThread = null;
        }
        if (arm2PickUPReadyThread != null){
            arm2PickUPReadyThread.interrupt();
            arm2PickUPReadyThread = null;
        }
        armPickUPThread = new ArmPickUPThread();
        armPickUPThread.start();
    }

    private class ArmPickUPThread extends Thread
    {

        //intialize the current encoder reading
        public ArmPickUPThread()
        {

            this.setName("Arm Pick Up Thread");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            armInCtrlTask = true;
            try
            {
                while ((!isInterrupted()) && (armInCtrlTask))
                {


                    robot.claw_open.setPosition(CLAW_OPEN_POS);
                    clawOpenFlag = true;
                    Thread.sleep(200);
                    arm_tilt_Pos = ARM_PICKUP_POS;
                    robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
                    if (robot.arm_tilt.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    robot.arm_tilt.setPower(arm_tilt_power);
                    if(Math.abs(robot.arm_tilt.getCurrentPosition() - arm_tilt_Pos) > 5){
                        Thread.sleep(20);
                    }
                    Thread.sleep(400);

                    robot.claw_open.setPosition(CLAW_CLOSE_POS);
                    clawOpenFlag = false;
                    Thread.sleep(400);
                    arm_tilt_Pos = ARM_PICKUP_MIDDLE_POS1;             // go to hold arm/stone position
                     robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
                    robot.arm_tilt.setPower(0.4);
                    while(Math.abs(robot.arm_tilt.getCurrentPosition() - arm_tilt_Pos) > 50){
                        Thread.sleep(20);
                    }
                    armInCtrlTask = false;

                    Thread.yield();

                }

            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {armInCtrlTask = false;}

        }
    }


    void arm2PickUpReadyPos() {
        if (arm2Level7PosThread != null){
            arm2Level7PosThread.interrupt();
            arm2Level7PosThread = null;
        }
        if (armPickUPThread != null){
            armPickUPThread.interrupt();
            armPickUPThread = null;
        }
        arm2PickUPReadyThread = new Arm2PickUPReadyThread();
        arm2PickUPReadyThread.start();
    }
    private class Arm2PickUPReadyThread extends Thread
    {

        //intialize the current encoder reading
        public Arm2PickUPReadyThread()
        {

            this.setName("Arm 2 Pick Up Ready Thread");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            armInCtrlTask = true;
            stretchInCtrlTask = true;
            try
            {
                while ((!isInterrupted()) && (armInCtrlTask))
                {


                    robot.claw_rotate.setPosition(0);
                    robot.claw_open.setPosition(CLAW_OPEN_POS);
                    ftcWait(400);
                    clawOpenFlag = true;
                    arm_tilt_Pos = ARM_DROP_LEVEL7 - 100;
                    robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
                    if (robot.arm_tilt.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    robot.arm_tilt.setPower(0.6);
                    while(Math.abs(robot.arm_tilt.getCurrentPosition() - arm_tilt_Pos) > 10){
                        Thread.sleep(20);
                    }
                    robot.claw_open.setPosition(CLAW_CLOSE_POS);        // close the claw

                    clawOpenFlag = false;

                    Thread.sleep(100);
                    arm_stretch_Pos = STRETCH_RESET_POS;
                    robot.arm_stretch.setTargetPosition(STRETCH_RESET_POS);
                    if (robot.arm_stretch.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                        robot.arm_stretch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    robot.arm_stretch.setPower(1.0);
                    arm_tilt_Pos = ARM_PICKUP_POS;
                    robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
                    robot.arm_tilt.setPower(1.0);
                    while(Math.abs(robot.arm_tilt.getCurrentPosition() - arm_tilt_Pos) > 50){

                        Thread.sleep(20);
                    }
                    robot.claw_rotate.setPosition(CLAW_ROTATION_INI);
                    robot.claw_open.setPosition(CLAW_OPEN_POS);
                    clawOpenFlag = true;
                    arm_tilt_Pos = ARM_PICK_READY_POS;
                    robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
                    robot.arm_tilt.setPower(0.5);
                    while(Math.abs(robot.arm_tilt.getCurrentPosition() - arm_tilt_Pos) > 30){

                        Thread.sleep(20);
                    }
                    armInCtrlTask = false;
                    stretchInCtrlTask = false;
                    Thread.yield();

                }

            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {armInCtrlTask = false;stretchInCtrlTask = false;}

        }
    }

    private void arm2Level7Pos(){
        if (arm2PickUPReadyThread != null){
            arm2PickUPReadyThread.interrupt();
            arm2PickUPReadyThread = null;
        }
        if (armPickUPThread != null){
            armPickUPThread.interrupt();
            armPickUPThread = null;
        }
        arm2Level7PosThread = new Arm2Level7PosThread();
        arm2Level7PosThread.start();
    }

    private class Arm2Level7PosThread extends Thread{

        //intialize the current encoder reading
        public Arm2Level7PosThread()
        {

            this.setName("Arm 2 Level 7 pos Thread");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            armInCtrlTask = true;
            stretchInCtrlTask = true;
            try
            {
                while ((!isInterrupted()) && (armInCtrlTask))
                {


                    robot.claw_open.setPosition(CLAW_CLOSE_POS);
                    clawOpenFlag = false;
                    robot.claw_rotate.setPosition(0);

                    arm_tilt_Pos = ARM_DROP_LEVEL7;
                    robot.arm_tilt.setTargetPosition(arm_tilt_Pos);
                    if (robot.arm_tilt.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                        robot.arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    robot.arm_tilt.setPower(0.5);
                    while(Math.abs(robot.arm_tilt.getCurrentPosition() - ARM_DROP_LEVEL7) >= 100){
                        if( (robot.arm_tilt.getCurrentPosition() > ARM_PICKUP_MIDDLE_POS2) && (arm_stretch_Pos > STRETCH_MIDDLE_POS )){

                            arm_stretch_Pos = STRETCH_MIDDLE_POS;
                            robot.arm_stretch.setTargetPosition(arm_stretch_Pos);
                            if (robot.arm_stretch.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                                robot.arm_stretch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            }
                            robot.arm_stretch.setPower(0.5);
                        }
                        Thread.sleep(20);

                    }


                    armInCtrlTask = false;
                    stretchInCtrlTask = false;

                    Thread.yield();

                }

            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {armInCtrlTask = false;stretchInCtrlTask = false;}

        }


    }
    private void putDownCapStone(){
        if (putDownCapStoneThread != null){
            putDownCapStoneThread.interrupt();
            putDownCapStoneThread = null;
        }

        putDownCapStoneThread = new PutDownCapStoneThread();
        putDownCapStoneThread.start();
    }

    private class PutDownCapStoneThread extends Thread{

        //intialize the current encoder reading
        public PutDownCapStoneThread()
        {

            this.setName("Put Down Cap Stone Thread");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            blnPutDownCapStone = true;

            try
            {
                while ((!isInterrupted()) && (blnPutDownCapStone))
                {
                    robot.capStone.setPosition(CAPSTONE_PUTDOWN);
                    ftcWait(500);
                    robot.capStone.setPosition(CAPSTONE_INI);
                    ftcWait(500);
                    blnPutDownCapStone = false;

                    Thread.yield();

                }

            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {armInCtrlTask = false;stretchInCtrlTask = false;}

        }


    }


    private void ftcWait(long ms){
        try {
            Thread.sleep(ms);
        }
        catch(Exception e){

        }
    }

}
