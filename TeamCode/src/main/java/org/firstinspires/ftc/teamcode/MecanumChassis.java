package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumChassis {
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor intake_left = null;
    public DcMotor intake_right = null;
    public DcMotorEx arm_tilt = null;
    public DcMotor arm_stretch = null;
    public Servo claw_rotate = null;
    public Servo claw_open = null;
    public Servo claw_tilt = null;

    public ColorSensor sensorColorLF = null;
    public DistanceSensor sensorDistanceLF = null;

    public ColorSensor sensorColorLR = null;
    public DistanceSensor sensorDistanceLR = null;

    public ColorSensor sensorColorRF = null;
    public DistanceSensor sensorDistanceRF = null;

    public ColorSensor sensorColorRR = null;
    public DistanceSensor sensorDistanceRR = null;

    public BNO055IMU imu;

    public DigitalChannel lowerSwitch = null;
    public DigitalChannel upperSwitch = null;


    public Servo autoLeftClawL = null;
    public Servo autoLeftClawU = null;
    public Servo autoRightClawL = null;
    public Servo autoRightClawU = null;

    public Servo leftHook1 = null;
    public Servo leftHook2 = null;
    public Servo rightHook1 = null;
    public Servo rightHook2 = null;

    public Servo capStone = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MecanumChassis(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //IMU
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "lf_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rf_drive");
        leftRearDrive  = hwMap.get(DcMotor.class, "lr_drive");
        rightRearDrive = hwMap.get(DcMotor.class, "rr_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        intake_left  = hwMap.get(DcMotor.class, "intake_left");
        intake_right = hwMap.get(DcMotor.class, "intake_right");
        arm_tilt  = hwMap.get(DcMotorEx.class, "arm_tilt");
        arm_stretch = hwMap.get(DcMotor.class, "arm_stretch");

        //intake
        intake_left.setDirection(DcMotor.Direction.FORWARD);
        intake_right.setDirection(DcMotor.Direction.REVERSE);
        arm_tilt.setDirection(DcMotor.Direction.FORWARD);
        arm_stretch.setDirection(DcMotor.Direction.FORWARD);




        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        intake_left.setPower(0);
        intake_right.setPower(0);
        arm_stretch.setPower(0);
        arm_tilt.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        intake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_stretch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_stretch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //servo
        // Define and initialize ALL installed servos.
        claw_rotate  = hwMap.get(Servo.class, "claw_rotate");
        claw_open = hwMap.get(Servo.class, "claw_open");
        claw_rotate.setPosition(0.4);     //middle position
        claw_open.setPosition(0.5);     //middle position




        claw_tilt  = hwMap.get(Servo.class, "claw_tilt");
        claw_tilt.setPosition(0.86);     //middle position


        //color sensor
        // Get a reference to our sensor object.
        // get a reference to the color sensor.
        sensorColorLF = hwMap.get(ColorSensor.class, "color_sensorLF");
        sensorColorLR = hwMap.get(ColorSensor.class, "color_sensorLR");
        // get a reference to the distance sensor that shares the same name.
        sensorDistanceLF = hwMap.get(DistanceSensor.class, "color_sensorLF");
        sensorDistanceLR = hwMap.get(DistanceSensor.class, "color_sensorLR");


        sensorColorRF = hwMap.get(ColorSensor.class, "color_sensorRF");
        sensorColorRR = hwMap.get(ColorSensor.class, "color_sensorRR");
        // get a reference to the distance sensor that shares the same name.
        sensorDistanceRF = hwMap.get(DistanceSensor.class, "color_sensorRF");
        sensorDistanceRR = hwMap.get(DistanceSensor.class, "color_sensorRR");

        // for magnetic switch sensor
        lowerSwitch = hwMap.get(DigitalChannel.class, "lowerSwitch");
        lowerSwitch.setMode(DigitalChannel.Mode.INPUT);
        //upperSwitch = hwMap.get(DigitalChannel.class, "upperSwitch");
        //upperSwitch.setMode(DigitalChannel.Mode.INPUT);

        //for auto left claw
        autoLeftClawL = hwMap.get(Servo.class, "leftClawL");
        autoLeftClawU = hwMap.get(Servo.class, "leftClawU");
        autoLeftClawU.setPosition(0.55);
        autoLeftClawL.setPosition(0.575);

        //for left hook
        leftHook1 = hwMap.get(Servo.class, "leftHookF");
        leftHook2 = hwMap.get(Servo.class,"leftHookR");
        leftHook1.setPosition(0.48);
        leftHook2.setPosition(0.49);

        //for auto right claw
        autoRightClawL = hwMap.get(Servo.class, "rightClawL");
        autoRightClawU = hwMap.get(Servo.class, "rightClawU");
        autoRightClawU.setPosition(0.44);
        autoRightClawL.setPosition(0.47);

        //for left hook
        rightHook1 = hwMap.get(Servo.class, "rightHookF");
        rightHook2 = hwMap.get(Servo.class,"rightHookR");
        rightHook1.setPosition(0.55);
        rightHook2.setPosition(0.472);

        capStone = hwMap.get(Servo.class, "capstone_servo");
        capStone.setPosition(0.55);

    }


}
