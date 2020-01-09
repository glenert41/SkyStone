package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareBACONbot;

import java.util.Locale;
//import org.firstinspires.ftc.teamcode.Teleops.HardwareMap;


@Autonomous(name = "Shields Mecanum Auto", group = "Opmode")
//@Disabled

public class ShieldsMecanumAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareShieldsMecanum robot = new HardwareShieldsMecanum();
    // === DEFINE CONSTANTS HERE! ===

    // ==============================
    public void runOpMode() {
        double lastTime = runtime.milliseconds();

        // State used for updating telemetry
        //Orientation angles;
        //Acceleration gravity;

        robot.init(hardwareMap);
        // Choosing the team color
        telemetry.addData("Greetings, Human", "");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // we don't want to change orientation, just heading
        Orientation targOrient;
        targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingPowerTime(90,0.5,3, targOrient);
        //stopDriving();
        //lastTime = runtime.milliseconds();
        //stopDriving();
    }
    // Functions ----------------------------------------------------------------------------------------------------------------

    void headingPowerTime(int heading, double pwr, double t, Orientation targ) {
        // this is a new branch :)

        /* pwr(sin(heading+45))+r [M4]------[M1] pwr(cos(heading+45))-r
                                    |        |
           pwr(cos(heading+45))+r [M3]------[M2] pwr(sin(heading+45))-r */

        Orientation currOrient;
        currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
        double targAng = 0.0;  // target.angleUnit.DEGREES.normalize(target.firstAngle);
        double error = targAng - currAng;

        double r = -error / 180 * (pwr * 0.3);

        if ((r < .07) && (r > 0)) {
            r = .07;
        } else if ((r > -.07) && (r < 0)) {
            r = -.07;
        }
        robot.frontLeftMotor.setPower(pwr + r);
        robot.backLeftMotor.setPower(-pwr + r); //Changing the order in which the wheels start
        robot.backRightMotor.setPower(-pwr + r);
        robot.frontRightMotor.setPower(pwr + r);
        double lastTime = runtime.milliseconds();
        while (runtime.milliseconds() < lastTime + 1000) {

        }
    }
    //Stop Driving - Kill power to all the motors
    void stopDriving() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
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