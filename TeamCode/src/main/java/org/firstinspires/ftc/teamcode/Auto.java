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


@Autonomous(name = "BACON: OLD Autonomous 19-20", group = "Opmode")
@Disabled

//name file AutonomousB
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareBACONbot robot = new HardwareBACONbot();   // Use BACONbot's hardware


    public void runOpMode() {
        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;
        int task = 0; //1 = mat 2 = stones
        int mat = 1;
        int stones = 2;

        double meetDistance = 860; //Distance from wall to the Blocks/Mat (CM From Wall (BackSensor))
        double lastTime = runtime.milliseconds();



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

        while (!gamepad1.x && !gamepad1.b) {
        }
        //This sets the strips of lights and the screen of the phones to the team color
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
            double bothYellow = 2;
            double grabPrepPhase = 0;
            double scanPhase = 0;
            double grabPhase = 0;

            robot.liftMotor.setPower(-1);
            while (robot.liftMotor.getCurrentPosition() > -2000) {
                    //do nothing}
                }
            robot.liftMotor.setPower(0.0);

            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < 675) && opModeIsActive()) {
                telemetry.addData("Status", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
                telemetry.update();

                //**Changed the number here- not sure its quite perfect yet but this is the best we have gotten
            }
            scanPhase = 1;
            stopDriving();
            sleep(100);
            //telemetry.addData("got there", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
            //telemetry.update();



            while(scanPhase == 1 /*&& (robot.colorSensorL.alpha() > 30) && (robot.colorSensorR.alpha() > 30)*/) {
                strafeRight(.3);   ///ALL STRAFES ARE INVERTED IN AUTONOMOUS
                        //STRAFE RIGHT IN THE AUTONOMOUS CODE IS STRAFE LEFT IN REAL LIFE
                        //sorry for the all caps, it's just important
                        //-Love, Graham
                //sleep(100);
            //We may need to change the alpha values to get consistent readings
                if ((robot.colorSensorL.alpha() > 30) && (robot.colorSensorR.alpha() > 30)) {
                    bothYellow = 1;
                    //both are yellow or air
                }
                //The next two are just extra cases if it isn't reading properly
                if ((robot.colorSensorL.alpha() < 30) && (robot.colorSensorR.alpha() > 30)) {
                    bothYellow = 1;
                    //left is black and right is yellow or air
                }
                if ((robot.colorSensorL.alpha() > 30) && (robot.colorSensorR.alpha() < 30)) {
                    bothYellow = 1;
                    //left is yellow or air and right is black
                }

                // If it's black then bothYellow is false
                if ((robot.colorSensorL.alpha() < 30) && (robot.colorSensorR.alpha() < 30)) {
                    bothYellow = 0;
                }

                //telemetry.addData("leftVal = ","leftVal = " + robot.colorSensorL.alpha());
                //telemetry.addData("rightVal = ","rightVal = " + robot.colorSensorR.alpha());
                telemetry.addData("bothYellowVal: ", "Yellow State: " + bothYellow);
                telemetry.update();


                if (bothYellow == 0) { //breaks the scan phase loop

                   telemetry.addData("SICK","I SEE A SKYSTONE");
                   telemetry.update();
                   scanPhase = 0;
                   grabPrepPhase = 1;
                   relativeLayout.post(new Runnable() {
                        public void run() {
                            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                            robot.blinkinLedDriver.setPattern(robot.pattern);
                        }
                    });


                }
            }

            while(grabPrepPhase == 1){
                driveBackwards();
                while ((robot.backDistance.getDistance(DistanceUnit.MM) > 625) && opModeIsActive()){

                }
                stopDriving();

                robot.liftMotor.setPower(1);
                while (robot.liftMotor.getCurrentPosition() < 0) {
                }
                robot.liftMotor.setPower(0.0);

                grabPrepPhase = 0;
                grabPhase = 1;
            }

            while(grabPhase == 1){
                stopDriving();
                driveForward();
                while ((robot.backDistance.getDistance(DistanceUnit.MM) < 675) && opModeIsActive()){

                }
                telemetry.addData("I stop","I have entered the Grab Phase");
                telemetry.update();
                robot.clawServo.setPosition(1);
            }





        }

        //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        if((task == mat) && (teamcolor == 2)){
            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) && opModeIsActive()) //drive to mat
            {
            }
            stopDriving();
            lastTime= runtime.milliseconds();
            strafeLeft(.3); //this actually makes it go right toward the center of the mat
            while (runtime.milliseconds()<lastTime+1000){

            }
            stopDriving();
            robot.matServoL.setPosition(freePos);
            robot.matServoR.setPosition(grabPos);
            sleep(1000); //We can edit this delay based on it we need more time or not
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
            sleep(1000); //this makes sure we don't knock the mat when we begin to go towards parking

            strafeRight(.6); //Actually left towards the skybridge
            //Senses the red tape under the skybridge and tells the robot to stop
            while (robot.colorSensorDown.red()<30&& opModeIsActive()) {
                telemetry.addData("Red  ", robot.colorSensorDown.red());
                telemetry.update();
            }
            stopDriving();
        }




        //BLUE SIDE sorry for the caps lol
        if((task == mat) && (teamcolor == 1)){
            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) && opModeIsActive()) //drive to mat
            {
            }
            stopDriving();
            lastTime= runtime.milliseconds();
            strafeRight(.3); //this actually makes it go left toward the center of the mat
            while (runtime.milliseconds()<lastTime+1000){

            }
            stopDriving();
            robot.matServoL.setPosition(freePos);
            robot.matServoR.setPosition(grabPos);
            sleep(1000); //We can edit this delay based on it we need more time or not

            driveBackwards();

            while ((robot.backDistance.getDistance(DistanceUnit.MM) > 150) && opModeIsActive()) //drivetomat
            {
                telemetry.addData("backing up", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            stopDriving();
            robot.matServoL.setPosition(grabPos);
            robot.matServoR.setPosition(freePos);
            sleep(1000); //this makes sure we don't knock the mat when we begin to go towards parking

            strafeLeft(.6); //Actually right towards the skybridge
            //Senses the BLUE tape under the skybridge and tells the robot to stop
            while (robot.colorSensorDown.blue()<30&& opModeIsActive()) {
                telemetry.addData("Blue  ", robot.colorSensorDown.blue());
                telemetry.update();
            }
            stopDriving();
        }

















        //TODO What does this part do? It isn't in the same spot as the other things with sensing skystones
        while (opModeIsActive()) {
            //telemetry.addData("Alpha", robot.colorSensorL.alpha());
            //telemetry.addData("Red  ", robot.colorSensorDown.red());
            if ((robot.colorSensorL.alpha() < 40) && (robot.colorSensorR.alpha() < 40))
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
 }
 // if skystons ARE Detected
 if (ColorBot.isBlack(robot.colorSensorL) && ColorBot.isBlack(robot.colorSensorR)) {
 telemetry.addData("Object is Black", robot.colorSensorL.red());
 //Detects the Skystone
 while (robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance + 20) {
 driveBackwards();
 }
 //pick up skystone //test
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
        strafeLeft(3); //Out from the parking tape under the skybridge
        sleep(1000);
        stopDriving();
        robot.clawServo.setPosition(1);    //Claw servo in the open position
        stopDriving();
        strafeRight(0.6);  //Back to the parking tape under the skybridge
        //Stop at the red tape
        while (robot.colorSensorDown.red()<30 && opModeIsActive() ) {

        }
        stopDriving();

    }


}
