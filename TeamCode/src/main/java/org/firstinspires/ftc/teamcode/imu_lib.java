package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
public class imu_lib{
    //robot hardware definitions
    robot_hardware robot;
    action_lib     action;

    //IMU overhead
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double lasterror;
    double ticks = 383.6/(Math.PI*10.2);
    double prev = 0, now = 0;


    public imu_lib(robot_hardware robot,action_lib action){
        this.robot = robot;
        this.action = action;
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {

        Orientation angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double getProportionalTerm(double heading, double kp, double kd)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle;

        angle = getAngle();
        double error = heading-angle;
        if (angle == heading)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * kp;

        double  derivative = lasterror-error;
        double Dcorrection = derivative*kd;


        lasterror = error;
        return correction+Dcorrection;

    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        int dir =1;


        if (degrees < globalAngle)
        {   // turn right.
            dir=1;
        }
        else if (degrees > globalAngle)
        {   // turn left.
            dir=-1;
        }
        else return;

        // set power to rotate.
        this.action.run_drive(power*dir,power*dir,power*dir,power*dir);

        // rotate until turn is completed.
        if (degrees < globalAngle)
        {
            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        this.action.stop_drive();
    }

    public void goStraightIMU(int direction, double heading, double cm, double power, robot_hardware robot){
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double strafe,forward;
        if (direction == 1){
            strafe = 0;
            forward = power;
        }else{
            strafe = power;
            forward = 0;
        }

        if(power>0) {
            while (java.lang.Math.abs(robot.rf_drive.getCurrentPosition()) < cm * ticks + prev) {
                double correction = getProportionalTerm(heading, 0.007, 0);
                robot.lf_drive.setPower(strafe - forward + correction);
                robot.rf_drive.setPower(strafe + forward + correction);
                robot.lr_drive.setPower(-strafe - forward + correction);
                robot.rr_drive.setPower(-strafe + forward + correction);
            }
            prev = robot.rf_drive.getCurrentPosition();
        }else{
            while (java.lang.Math.abs(robot.rf_drive.getCurrentPosition()) > cm * ticks + prev) {
                double correction = getProportionalTerm(heading, 0.007, 0);
                robot.lf_drive.setPower(strafe - forward - correction);
                robot.rf_drive.setPower(strafe + forward - correction);
                robot.lr_drive.setPower(-strafe - forward - correction);
                robot.rr_drive.setPower(-strafe + forward - correction);
            }
            prev = robot.rf_drive.getCurrentPosition();
        }

        robot.lr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rr_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lf_drive.setPower(0);
        robot.rf_drive.setPower(0);
        robot.lr_drive.setPower(0);
        robot.rr_drive.setPower(0);
    }

}