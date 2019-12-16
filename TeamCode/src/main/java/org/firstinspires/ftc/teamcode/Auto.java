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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareBACONbot;
//import org.firstinspires.ftc.teamcode.Teleops.HardwareMap;


@Autonomous(name = "BACON: Autonomous 19-20", group = "Opmode")
//@Disabled

//name file AutonomousB
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareBACONbot robot = new HardwareBACONbot();   // Use a BACONbot's hardware


    public void runOpMode() {
        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;
        int task = 0; //1 = mat 2 = stones
        int mat = 1;
        int stones = 2;

        double meetDistance = 860; //Distance from wall to the Blocks/Mat (CM From Wall (BackSensor))



        float grabPos = 0;  //change these later (written 12-3-19)
        float freePos = 1;  //change these later  (written 12-3-19)
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        robot.init(hardwareMap);
        // Choosing the team color
        telemetry.addData("Press X for Blue, B for Red", "");
        telemetry.update();
        // TODO: What will go in this while loop?
        while (!gamepad1.x && !gamepad1.b) {
        }
        if (gamepad1.x) {
            teamcolor = blue;
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLUE);
                    robot.pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
                    robot.blinkinLedDriver.setPattern(robot.pattern);
                }
            });
        }
        if (gamepad1.b) {
            teamcolor = red;
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                    robot.pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                    robot.blinkinLedDriver.setPattern(robot.pattern);
                }
            });
        }
        telemetry.addData("teamcolor ", teamcolor);
        telemetry.update();

        // Choosing the task
        telemetry.addData("Press A for mat, Y for stones", "");
        telemetry.update();
        while (!gamepad1.a && !gamepad1.y) {
        }
        if (gamepad1.a) {
            task = mat;
        }
        if (gamepad1.y) {
            task = stones;
        }
        telemetry.addData("task ", task);
        telemetry.update();

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) robot.backDistance;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)


        //Stones --------------------------------------------------------------------------++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if (task == stones) {

            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) && opModeIsActive()) {
                telemetry.addData("Status", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
                telemetry.update();
                //**Changed the number here- not sure its quite perfect yet but this is the best we have gotten
            }
            stopDriving();
            telemetry.addData("got there", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
            telemetry.update();




        }
        if(task == mat){
            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) && opModeIsActive()) //drivetomat
            {

            }
            stopDriving();
            robot.matServoL.setPosition(freePos);
            robot.matServoR.setPosition(grabPos);
            sleep(1500);
            //grabmat
            //drivebacktowall
            //releasemat
            //gotored

            driveBackwards();

            while ((robot.backDistance.getDistance(DistanceUnit.MM) > 200) && opModeIsActive()) //drivetomat
            {
                telemetry.addData("backing up", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            stopDriving();
            robot.matServoL.setPosition(grabPos);
            robot.matServoR.setPosition(freePos);

            strafeRight(.6);
            while (robot.colorSensorDown.red()<30&& opModeIsActive()) {
                telemetry.addData("Red  ", robot.colorSensorDown.red());
                telemetry.update();
            }
            stopDriving();
        }

        while (opModeIsActive()) {
            telemetry.addData("Alpha", robot.colorSensorL.alpha());
            telemetry.addData("Red  ", robot.colorSensorDown.red());
            if (robot.colorSensorL.alpha() < 80)
                telemetry.addData("Skystone", 1);
            else
                telemetry.addData("Yellow", 0);
            telemetry.update();
        }
/***********
 /* Sample code for taking yellow/black readings
 //ColorSensor bottomColorSensor;
 // bottomColorSensor = hardwareMap.colorSensor.get("bCS");
 //if NO skystone detected
 if (ColorBot.isYellow(robot.colorSensorL) && ColorBot.isYellow(robot.colorSensorR)) {
 //strafe left
 strafeLeft(3);
 sleep(1000);
 }
 // if skystons ARE Detected
 if (ColorBot.isBlack(robot.colorSensorL) && ColorBot.isBlack(robot.colorSensorR)) {
 telemetry.addData("Object is Black", robot.colorSensorL.red());
 //Detects the Skystone
 while (robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance + 20) {
 driveBackwards();
 }
 //pick up skystone
 stopDriving();
 robot.clawServo.setPosition(grabPos); //sets the servo to Grab Position
 while (robot.backDistance.getDistance(DistanceUnit.MM) > meetDistance - 20) {
 driveForward();
 }
 while (!ColorBot.isRed(robot.colorSensorDown)) {
 strafeRight(.3);
 }
 */
        //stopDriving();

        //outAndBack();

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

    //Drive Backwards - Used for starting the game
    void driveBackwards() {
        robot.frontLeftMotor.setPower(-0.5);
        robot.frontRightMotor.setPower(0.5);
        robot.backLeftMotor.setPower(-0.5);
        robot.backRightMotor.setPower(0.5);

    }

    //Drive Forwards - Towards where the Backsensor is facing
    void driveForward() {
        robot.frontLeftMotor.setPower(0.5);
        robot.frontRightMotor.setPower(-0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.backRightMotor.setPower(-0.5);
    }

    //Strafe Left - (used to strafe towards the center line for parking)
    void strafeLeft(double pwr) {  //added int pwr to reduce initial power
        robot.frontLeftMotor.setPower(pwr);
        robot.backRightMotor.setPower(-pwr); //Changing the order in which the wheels start
        robot.frontRightMotor.setPower(pwr);
        robot.backLeftMotor.setPower(-pwr);

    }

    void strafeRight(double pwr) {  //added int pwr to reduce initial power
        robot.frontLeftMotor.setPower(-pwr);
        robot.backRightMotor.setPower(pwr); //Changing the order in which the wheels start
        robot.frontRightMotor.setPower(-pwr);
        robot.backLeftMotor.setPower(pwr);

    }

    void outAndBack() {
        strafeLeft(3);
        sleep(1000);
        stopDriving();
        //robot.clawServo.setPosition(1);        //UPDATE THIS NUMBER TO WHATEVER freePOS is
        stopDriving();
        while (!ColorBot.isRed(robot.colorSensorDown)) {
            strafeRight(0.3);
        }


    }


}
