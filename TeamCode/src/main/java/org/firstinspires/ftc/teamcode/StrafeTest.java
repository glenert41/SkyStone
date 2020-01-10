// FTC Team 7080 BACON
// Autonomous code for red & blue side, Stone & Mat side

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
//import org.firstinspires.ftc.teamcode.Teleops.HardwareMap;


@Autonomous(name = "BACON: StrafeDog", group = "Opmode")
@Disabled

//name file AutonomousB
public class StrafeTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareBACONbot robot = new HardwareBACONbot();   // Use BACONbot's hardware
    // === DEFINE CONSTANTS HERE! ===
    double STRAFE_SPEED = 0.3;  // Motor power global variable

    // ==============================
    public void runOpMode() {
        // State used for updating telemetry
        //Orientation angles;
        //Acceleration gravity;
        robot.init(hardwareMap);
        // Choosing the team color
        telemetry.addData("strafe test!", "");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double lastTime = runtime.milliseconds();

        Orientation targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (runtime.milliseconds() < 60000 && opModeIsActive()) {
            /*double now = runtime.milliseconds();
            if(now > lastTime + 10) {
                strafe(STRAFE_SPEED, targOrient, -1);
                lastTime = now;
            }
            */
            strafe(STRAFE_SPEED, targOrient, -1);
        }
        stopDriving();
    }

    // Functions ----------------------------------------------------------------------------------------------------------------


    //Driving Functions

    //Stop Driving - Kill power to all the motors
    void stopDriving() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    void strafe(double pwr, Orientation target, int dir) {  //added int pwr to reduce initial power
        //Get the current orientation
        Orientation currOrient;
        currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Compare the current orientation to the target
        double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
        double targAng = 0.0;  // target.angleUnit.DEGREES.normalize(target.firstAngle);
        double error = targAng - currAng;
        telemetry.addData("error: ", error);
        telemetry.addData("backdistance:", robot.backDistance.getDistance(DistanceUnit.MM));
        telemetry.update();
        //scale the error so that it is a motor value and
        //then scale it by a third of the power to make sure it
        //doesn't dominate the movement
        double r = -error / 180 * (pwr * 10);

        //if the absolute value of r is less than
        //.07, the motors won't do anything, so if
        //it is less than .07, make it .07
        /*if ((r < .07) && (r > 0)) {
            r = .02;
        } else if ((r > -.07) && (r < 0)) {
            r = -.02;
        }*/
        //send the power to the motors
        robot.frontLeftMotor.setPower(pwr + r);
        robot.backLeftMotor.setPower(-pwr + r);
        robot.backRightMotor.setPower(-pwr + r);
        robot.frontRightMotor.setPower(pwr + r);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
