
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VufroriaNavigation;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * this is a very simple teleop code and should not be used for competition
 */
@Autonomous(name = "vision")
public class visionAuto extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "ARSsLcv/////AAABmaPnuXwWvUUcmEwKRJUD7zsqO7JIqriiHhFyZBocTTTMF8T8EA4mCbJtMqnxh1TufzQXUOapLLMgLOG9+pJ77k4LT2uFLHXqlvu6yEXSpXpYi2xtaMfGHYOnxiDtXXjp+1BUc/jZBGgET0URPPPu1HXwGy8MSHS5PDM7ZlZobnMSAuHZFKjue5KYUHBHe4QBbZ1/S9ybpA33GNHpcwK3NPAI0jeXkrovdvBDq0fE56lMN7xTsGOKcQWf8KdpKhWdvS9lzd2u1mGbNontyiXJVyKSC5E7Vr4wszt68uiSCPEy4kWZ0eh+5S0MmgJZprRo7SX4s9tSXCmuU+bceuu2X8kXZAzwX78EtkqrEe1bm3O+";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap,telemetry);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//hold the arm up
        telemetry.addLine("done calibration, ready to start");
        telemetry.update();
        waitForStart();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VufroriaNavigation nav = new VufroriaNavigation(vuforia,parameters);

        while(true){
            nav.track(telemetry);
        }
        }
    }
