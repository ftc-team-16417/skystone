package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Drive Imu")
public class IMU extends LinearOpMode {

    Drive drive =  new Drive(hardwareMap);
}
    /*DcMotor leftMotor, rightMotor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .70, correction;
*/
    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        drive.doDrive(joy.x, joy.y);
    }