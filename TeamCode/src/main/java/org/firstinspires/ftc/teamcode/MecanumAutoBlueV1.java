package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

//import org.firstinspires.ftc.teamcode.MecanumAutoDrive;
//import org.firstinspires.ftc.teamcode.MecanumChassis;


@Autonomous(name="Blue Side Auto V1")
//@Disabled


public class MecanumAutoBlueV1 extends LinearOpMode {
    /* Declare OpMode members. */
    MecanumChassis robot   = new MecanumChassis();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    // State used for updating telemetry
    volatile Orientation angles;        //just for telemetry
    Acceleration gravity;


    //Mecanum auto drive class
    MecanumAutoDrive mecanumAutoDrive = null;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValuesF[] = {0F, 0F, 0F};
    float hsvValuesR[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValuesF;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    final double LEFT_HOOK1_INI = 0.48;
    final double LEFT_HOOK2_INI = 0.49;
    final double LEFT_HOOK_RANGE = 0.333;
    final double RIGHT_HOOK1_INI = 0.55;
    final double RIGHT_HOOK2_INI = 0.472;
    final double RIGHT_HOOK_RANGE = 0.403;


    final double LEFT_CLAWU_INI = 0.55;
    final double LEFT_CLAWL_INI = 0.575;
    final double LEFT_CLAWU_PICK = LEFT_CLAWU_INI - 0.2 ;
    final double LEFT_CLAWL_PICK = LEFT_CLAWL_INI - 0.34;
    final double LEFT_CLAWU_HOLD = LEFT_CLAWU_INI + 0.05;
    final double LEFT_CLAWL_HOLD = LEFT_CLAWL_INI -  0.1;
    final double LEFT_CLAWU_RELEASE = LEFT_CLAWU_INI - 0.13;
    final double RIGHT_CLAWU_INI = 0.44;
    final double RIGHT_CLAWL_INI = 0.47;
    final double RIGHT_CLAWU_PICK = RIGHT_CLAWU_INI + 0.2 ;
    final double RIGHT_CLAWL_PICK = RIGHT_CLAWL_INI + 0.34;
    final double RIGHT_CLAWU_HOLD = RIGHT_CLAWU_INI - 0.05;
    final double RIGHT_CLAWL_HOLD = RIGHT_CLAWL_INI +  0.05;
    final double RIGHT_CLAWU_RELEASE = RIGHT_CLAWU_INI + 0.13;

    final double DIS_TH = 30;        //hsv value
    final double STONE_WIDTH = 0.2 - 0.04;     // 20cm = 0.2m
    int stoneCheck = 0;
    double diffDis = 0;

    SkystoneNavigation skystoneNavigaitonThread = null;
    VectorF translation = null;
    Orientation rotation = null;

    double OFFSETY_2 = 0.83;   // for red side
    double OFFSETX_2 = -1.25;
    double OFFSET_Y = 0.19;
    double offsetY = 0;
    double OFFSET_X = 0;
    double offsetX = 0;
    double offsetAngle = 0;

    double OFFSETY_1 = 0.72;
    double OFFSETX_1 = -1.15;

    @Override
    public void runOpMode()  throws InterruptedException{
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while(!isStopRequested() && ! robot.imu.isGyroCalibrated() &&
                                    ! robot.imu.isAccelerometerCalibrated() &&
                                    ! robot.imu.isMagnetometerCalibrated() &&
                                    ! robot.imu.isSystemCalibrated())
        {
            idle();
        }


        robot.autoLeftClawU.setPosition(LEFT_CLAWU_INI);
        robot.autoLeftClawL.setPosition(LEFT_CLAWL_INI);
        robot.autoRightClawL.setPosition(RIGHT_CLAWL_INI);
        robot.autoRightClawU.setPosition(RIGHT_CLAWU_INI);

        robot.leftHook1.setPosition(LEFT_HOOK1_INI);
        robot.leftHook2.setPosition(LEFT_HOOK2_INI);
        robot.rightHook2.setPosition(RIGHT_HOOK2_INI);
        robot.rightHook1.setPosition(RIGHT_HOOK1_INI);


        composeTelemetry();
        telemetry.update();

        skystoneNavigaitonThread = new SkystoneNavigation(robot, telemetry);


        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();
        mecanumAutoDrive = new MecanumAutoDrive(robot, telemetry);


        skystoneNavigaitonThread.start();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        /*
        pickUpStone();
        sleep(2000);
        dropOffStone();
*/

        mecanumAutoDrive.straferTask(0.72,0,0.2,0.05,5);


        //check position by back camera for front perimeter 1
        sleep(300);
        offsetY = 0.03;
      //  checkPosition1();


        stoneCheck = checkColor();

        if (stoneCheck == 0) {
            //middle stone is sky stone
           // mecanumAutoDrive.straferTask(0.04,0,-0.05,0.05,5);

        }
        else if(stoneCheck == -1) {
            //front one
            mecanumAutoDrive.goStraightTask(STONE_WIDTH, 0, 0.1, 0.02, 5);
          //  mecanumAutoDrive.straferTask(0.04,0,-0.05,0.05,5);

        }
        else {
            //back one
            mecanumAutoDrive.goStraightTask(STONE_WIDTH, 0, -0.1, 0.02, 5);
          //  mecanumAutoDrive.straferTask(0.04,0,-0.05,0.05,5);

        }

        if (offsetY > 0){
            mecanumAutoDrive.straferTask(Math.abs(offsetY), 0,0.2,0,5);
        }
        pickUpStone();

        mecanumAutoDrive.straferTask(0.21,0,-0.2,0.05,5);

        mecanumAutoDrive.goStraightTask(2.2 + 0.05 + stoneCheck * STONE_WIDTH, 1, 0.35, 0.02, 5);

     //   robot.autoLeftClawL.setPosition(LEFT_CLAWL_INI - 0.2);
        mecanumAutoDrive.straferTask(0.29,1,0.2,0.05,5);
        //dropOffStone

        robot.autoRightClawU.setPosition(RIGHT_CLAWU_RELEASE);
        ftcWait(100);
        robot.autoRightClawL.setPosition(RIGHT_CLAWL_HOLD + 0.02);
        ftcWait(300);

        mecanumAutoDrive.straferTask(0.21,0,-0.2,0.05,5);
        //reset pick up arm
        robot.autoRightClawU.setPosition(RIGHT_CLAWU_INI);
        robot.autoRightClawL.setPosition(RIGHT_CLAWL_INI);

        //mecanumAutoDrive.goStraightTask(2.2 - 0.07 + 5 * STONE_WIDTH + stoneCheck * STONE_WIDTH, 0, -0.35, 0.02, 5);
        mecanumAutoDrive.goStraightTask(2.62, 0, -0.35, 0.02, 5);

        correctRobotPositionByCam();
        offsetX = 0;
        offsetY = 0;
        if (stoneCheck == -1) {
            if (offsetX > 0) {
                mecanumAutoDrive.goStraightTask(offsetX, 0, -0.1, 0.02, 5);
            }
            if (offsetX < 0){
                mecanumAutoDrive.goStraightTask(offsetX, 0, 0.1, 0.02, 5);
            }
        }
        else if(stoneCheck == 0){
            mecanumAutoDrive.goStraightTask(STONE_WIDTH + offsetX, 0, -0.1, 0.02, 5);
        }
        else {
            mecanumAutoDrive.goStraightTask(2 * STONE_WIDTH + offsetX + 0.03, 0, -0.1, 0.02, 5);
        }

        if (Math.abs(offsetY) > 0.05){
            mecanumAutoDrive.straferTask(OFFSET_Y,0,0.2,0.05,5);
        }
        else{
            mecanumAutoDrive.straferTask(OFFSET_Y + offsetY,0,0.2,0.05,5);
        }


        pickUpStone();

        mecanumAutoDrive.straferTask(OFFSET_Y + 0.03,0,-0.2,0.05,5);

        mecanumAutoDrive.goStraightTask(2.50 + (1 + stoneCheck) * STONE_WIDTH, -2, 0.35, 0.02, 5);

     //   robot.autoLeftClawL.setPosition(LEFT_CLAWL_INI - 0.2);
        mecanumAutoDrive.straferTask(0.25,-2,0.2,0.05,5);
        //drop off stone
        robot.autoRightClawU.setPosition(RIGHT_CLAWU_RELEASE);
        ftcWait(100);
        robot.autoRightClawL.setPosition(RIGHT_CLAWL_HOLD + 0.02);
        ftcWait(300);
        //hookdown first
        robot.rightHook1.setPosition(RIGHT_HOOK1_INI - RIGHT_HOOK_RANGE);
        robot.rightHook2.setPosition(RIGHT_HOOK2_INI + RIGHT_HOOK_RANGE);
        mecanumAutoDrive.driveRobot(0.15,-0.15,-0.15,0.15);
        ftcWait(400);
        //reset pick up arm
        robot.autoRightClawU.setPosition(RIGHT_CLAWU_INI);
        robot.autoRightClawL.setPosition(RIGHT_CLAWL_INI);

        //mecanumAutoDrive.driveRobot(-0.3,0.3,0.3,-0.3);

        mecanumAutoDrive.driveRobot(-0.1,0.01,0.01,-0.1);
        ftcWait(400);
        mecanumAutoDrive.driveRobot(-0.3,0.05,0.05,-0.3);
        ftcWait(1600);
        mecanumAutoDrive.turnRobotTask(90,0.5,10.0, MecanumAutoDrive.TURN_METHOD.TWO_WHEEL, 5);




        //mecanumAutoDrive.driveRobot(0.3,0.3,0.3,0.3);
        //ftcWait(1000);
        mecanumAutoDrive.goStraightTask(0.7, 90, 0.25, 0.02, 5);

        rightHookUp();
        mecanumAutoDrive.driveRobot(0.3,-0.3,-0.3,0.3);

        ftcWait(600);
        mecanumAutoDrive.stopRobot();
        robot.intake_left.setPower(-0.7);
        robot.intake_right.setPower(-0.7);
        mecanumAutoDrive.goStraightTask(0.53, 90, -0.35, 0.02, 5);
        mecanumAutoDrive.straferTask(0.92,90,-0.2,0.05,5);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);

        while(true) {
            if (skystoneNavigaitonThread.targetVisible){
               correctRobotPositionByCam();
               // checkPosition1();
                try {
/*
                    if (skystoneNavigaitonThread.targetName == "Front Perimeter 1"){
                        telemetry.addData("Visible Target", skystoneNavigaitonThread.targetName);
                    }
                    if (skystoneNavigaitonThread.getPosition) {

                        telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                skystoneNavigaitonThread.translation.get(0), skystoneNavigaitonThread.translation.get(1), skystoneNavigaitonThread.translation.get(2));
                    }


                    if (skystoneNavigaitonThread.getOrientation) {

                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", skystoneNavigaitonThread.rotation.firstAngle, skystoneNavigaitonThread.rotation.secondAngle, skystoneNavigaitonThread.rotation.thirdAngle);
                    }
*/
                    telemetry.addData( "IMU angle ", " %.0f", mecanumAutoDrive.getHeadingAngle() );
                    telemetry.addData( "Offset Y ", "%.0f", offsetY);
                   telemetry.addData( "Offset Angle ","%.0f",offsetAngle );


                }catch (Exception e){
                    telemetry.addData( "Sth wrong", "wong" );
                }
                telemetry.update();
            }

        }


/*
        while(true){
            mecanumAutoDrive.getHeadingAngle();

            // Read the sensor
            Color.RGBToHSV((int) (robot.sensorColorLF.red() * SCALE_FACTOR),
                    (int) (robot.sensorColorLF.green() * SCALE_FACTOR),
                    (int) (robot.sensorColorLF.blue() * SCALE_FACTOR),
                    hsvValuesF);

            Color.RGBToHSV((int) (robot.sensorColorLR.red() * SCALE_FACTOR),
                    (int) (robot.sensorColorLR.green() * SCALE_FACTOR),
                    (int) (robot.sensorColorLR.blue() * SCALE_FACTOR),
                    hsvValuesR);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance1 (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistanceLF.getDistance(DistanceUnit.CM)));
            telemetry.addData("Distance2 (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistanceLR.getDistance(DistanceUnit.CM)));

            telemetry.addData("Stone Position",
                    String.format(Locale.US, "%d", stoneCheck));

            telemetry.addData("Diff Dis",
                    String.format(Locale.US, "%.02f", diffDis));

            //telemetry.addData("Alpha", robot.sensorColorLF.alpha());
            //telemetry.addData("Red  ", robot.sensorColorLF.red());
            //telemetry.addData("Green", robot.sensorColorLF.green());
            //telemetry.addData("Blue ", robot.sensorColorLF.blue());
            //telemetry.addData("HF", hsvValuesF[0]);
            //telemetry.addData("SF", hsvValuesF[1]);
            //telemetry.addData("VF", hsvValuesF[2]);

            //telemetry.addData("HR", hsvValuesR[0]);
            //telemetry.addData("SR", hsvValuesR[1]);
            //telemetry.addData("VR", hsvValuesR[2]);


            //telemetry.addData("HeadingF=","%.1f",mecanumAutoDrive.getHeadingAngle());


            //telemetry.update();
           //idle();


        }
*/
/*
        telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.update();
*/
    }


        //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);


                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        /*
        telemetry.addLine()
                .addData("vel", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getVelocity().toString();
                    }
                })
                .addData("pos", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getPosition().toString();
                    }
                });
        */

    /*
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
                */

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    private int checkColor(){
        Color.RGBToHSV((int) (robot.sensorColorRF.red() * SCALE_FACTOR),
                (int) (robot.sensorColorRF.green() * SCALE_FACTOR),
                (int) (robot.sensorColorRF.blue() * SCALE_FACTOR),
                hsvValuesF);

        Color.RGBToHSV((int) (robot.sensorColorRR.red() * SCALE_FACTOR),
                (int) (robot.sensorColorRR.green() * SCALE_FACTOR),
                (int) (robot.sensorColorRR.blue() * SCALE_FACTOR),
                hsvValuesR);

        diffDis = Math.abs(hsvValuesF[0]- hsvValuesR[0]);
        if ( diffDis < DIS_TH){
            return 1;  //back one
        }
        else{
            if (hsvValuesR[0] > hsvValuesF[0]){
                return -1;   // front
            }
            else{
                return 0;     //middle
            }
        }


    }

    private void pickUpStone(){
        try
        {
                robot.autoRightClawU.setPosition(RIGHT_CLAWU_PICK);
                sleep(200);
                robot.autoRightClawL.setPosition(RIGHT_CLAWL_PICK);

                Thread.sleep(500);
                robot.autoRightClawU.setPosition(RIGHT_CLAWU_HOLD);
                Thread.sleep(700);
                robot.autoRightClawL.setPosition(RIGHT_CLAWL_HOLD);
                Thread.sleep(400);
                //Thread.yield();

        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        //catch (InterruptedException e) {goStraightEndFlag = true;}
        // an error occurred in the run loop.
        catch (Exception e) {}
    }

    private void dropOffStone(){
        try
        {

            robot.autoRightClawU.setPosition(RIGHT_CLAWU_RELEASE);
            Thread.sleep(500);
            robot.autoRightClawU.setPosition(RIGHT_CLAWU_INI);
            Thread.sleep(300);
            robot.autoRightClawL.setPosition(RIGHT_CLAWL_INI);
            Thread.sleep(400);
            Thread.yield();

        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        //catch (InterruptedException e) {goStraightEndFlag = true;}
        // an error occurred in the run loop.
        catch (Exception e) {}
    }

    private void leftHookDown(){
        robot.leftHook1.setPosition(LEFT_HOOK1_INI + LEFT_HOOK_RANGE);
        robot.leftHook2.setPosition(LEFT_HOOK2_INI - LEFT_HOOK_RANGE);
        try {
            Thread.sleep(500);
        }
        catch(Exception e){
            robot.leftHook1.setPosition(LEFT_HOOK1_INI);
            robot.leftHook2.setPosition(LEFT_HOOK2_INI);
        }

    }

    private void rightHookDown(){
        robot.rightHook1.setPosition(RIGHT_HOOK1_INI - RIGHT_HOOK_RANGE);
        robot.rightHook2.setPosition(RIGHT_HOOK2_INI + RIGHT_HOOK_RANGE);
        try {
            Thread.sleep(500);
        }
        catch(Exception e){
            robot.rightHook1.setPosition(RIGHT_HOOK1_INI);
            robot.rightHook2.setPosition(RIGHT_HOOK2_INI);
        }

    }

    private void rightHookUp(){
        robot.rightHook1.setPosition(RIGHT_HOOK1_INI);
        robot.rightHook2.setPosition(RIGHT_HOOK2_INI);
        try {
            Thread.sleep(200);
        }
        catch(Exception e){
            robot.rightHook1.setPosition(RIGHT_HOOK1_INI);
            robot.rightHook2.setPosition(RIGHT_HOOK2_INI);
        }
    }

    private void ftcWait(long ms){
        try {
            Thread.sleep(ms);
        }
        catch(Exception e){

        }
    }

    private void correctRobotPositionByCam(){
        offsetY = 0;
        double angle = 0;
        boolean getData = false;
        int tryCnt = 0;
        while(!getData){
            tryCnt ++;
            if (tryCnt > 4){
                getData = true;     //give up, use default offset value
                offsetY = 0;
                offsetX = 0;
                offsetAngle = 0;
            }
            if (skystoneNavigaitonThread.targetVisible) {
                try {
                    if (skystoneNavigaitonThread.targetName == "Front Perimeter 2") {
                        if (skystoneNavigaitonThread.getPosition) {
                            telemetry.addData("Pos2 (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                    skystoneNavigaitonThread.translation.get(0), skystoneNavigaitonThread.translation.get(1), skystoneNavigaitonThread.translation.get(2));
                            offsetY =  (skystoneNavigaitonThread.translation.get(1)/1000.0f - OFFSETY_2);

                            offsetX =  (skystoneNavigaitonThread.translation.get(0)/1000.0f - OFFSETX_2);
                            telemetry.addData("X offset", "%.3f", offsetX);
                            telemetry.addData("Y offset", "%.3f", offsetY);
                            getData = true;
                        }


                        if (skystoneNavigaitonThread.getOrientation) {
                            angle = skystoneNavigaitonThread.rotation.thirdAngle;
                            if (angle > 0) offsetAngle = angle - 180;
                            if (angle < 0) offsetAngle = angle + 180;

                        }
                        telemetry.update();
                    }
                } catch (Exception e) {
                    offsetAngle = 0;
                    offsetY = 0;
                    offsetX = 0;

                }
            }

        }
    }

    private void checkPosition1(){
        offsetY = 0;
        double angle;
        boolean getData = false;
        int tryCnt = 0;
        while(!getData){
            tryCnt ++;
            if (tryCnt > 4){
                getData = true;     //give up, use default offset value
                offsetY = 0;
                offsetX = 0;
                offsetAngle = 0;
            }
            if (skystoneNavigaitonThread.targetVisible) {
                try {
                    if (skystoneNavigaitonThread.targetName == "Front Perimeter 2") {
                        if (skystoneNavigaitonThread.getPosition) {
                            telemetry.addData("Pos1 (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                    skystoneNavigaitonThread.translation.get(0), skystoneNavigaitonThread.translation.get(1), skystoneNavigaitonThread.translation.get(2));
                            offsetY = (skystoneNavigaitonThread.translation.get(1)/1000.0f - OFFSETY_1);

                            offsetX =  (skystoneNavigaitonThread.translation.get(0)/1000.0f - OFFSETX_1);
                            telemetry.addData("X offset", "%.3f", offsetX);
                            telemetry.addData("Y offset", "%.3f", offsetY);
                            getData = true;
                        }


                        if (skystoneNavigaitonThread.getOrientation) {
                            angle = skystoneNavigaitonThread.rotation.thirdAngle;
                            if (angle > 0) offsetAngle = angle - 180;
                            if (angle < 0) offsetAngle = angle + 180;

                        }
                        telemetry.update();
                    }
                } catch (Exception e) {
                    offsetAngle = 0;
                    offsetY = 0;
                    offsetX = 0;

                }
            }
            ftcWait(50);

        }
        /*
        if(getData){
            if (offsetY > 0){
                mecanumAutoDrive.straferTask(Math.abs(offsetY), 0,-0.2,0,5);
            }

        }*/
    }

}
