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


@Autonomous(name = "BACON: Test Autonomous 19-20", group = "Opmode")
//@Disabled

//name file AutonomousB
public class AutoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareBACONbot robot = new HardwareBACONbot();   // Use BACONbot's hardware
    // === DEFINE CONSTANTS HERE! ===
    double STRAFE_SPEED = 0.2;
    double FAST_SPEED = 1.0;
    double SLOW_SPEED = 0.2;
    int BLUETAPE = 22; // Blue tape down sensor color value
    int REDTAPE = 30; // Red tape down sensor color value

    // ==============================
    public void runOpMode() {
        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 0;
        int task = 0; //1 = mat 2 = stones
        int mat = 1;
        int stones = 2;
        double meetDistance = 860; //Distance from wall to the Blocks/Mat (CM From Wall (BackSensor))
        double lastTime = runtime.milliseconds();

        float grabPos = 0;  //change these later (written 12-3-19)
        float freePos = 1;  //change these later  (written 12-3-19)


        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        robot.init(hardwareMap);
        // Choosing the team color
        telemetry.addData("Press X for Blue, B for Red", "");
        telemetry.update();
        openClaw();

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
        //First troubleshooting steps for this section would be to check the direction of the strafes in scan and grab
        if ((task == stones) && (teamcolor == red)) {
            //This lifts the claw one level so that it won't be in the way of the blocks while scanning
            raiseClaw();
            //This gets the robot in the proper place to sense the Skystones
            positionRobot();
            //This performs the scanning operation until we find a Skystone
            scan(red); //1 means red
            //Setting it up to go up and grab the Skystone
            grabPrep();
            //Pick up the Skystone
            grabStone();
            //rotate to face mat side
            rotateR(-85.0, 0.3);
            //Park on the tape
            parkStonesRed();
            //go drop the stone in the build zone and return to the parking line
            outAndBackRed();
            stopDriving();

        }
        if ((task == stones) && (teamcolor == blue)) {
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            robot.blinkinLedDriver.setPattern(robot.pattern);
            //This lifts the claw one level so that it won't be in the way of the blocks while scanning
            raiseClaw();
            //This gets the robot in the proper place to sense the Skystones
            positionRobot();
            //This performs the scanning operation until we find a Skystone
            scan(blue); //2 means blue
            //Setting it up to go up and grab the Skystone
            grabPrep();
            //Pick up the Skystone
            grabStone();
            //rotate to face mat side
            rotateL(85.0, 0.3);
            //Park on the tape
            parkStonesBlue();
            //go drop the stone in the build zone and return to the parking line
            outAndBackBlue();
            stopDriving();

        }

        //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //BLUE TEAM MAT
        if ((task == mat) && (teamcolor == blue)) {
            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) && opModeIsActive()) //drive to mat
            {
            }
            stopDriving();
            lastTime = runtime.milliseconds();
             //this actually makes it go left toward the center of the mat
            Orientation targOrient;
            targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (runtime.milliseconds() < lastTime + 1000) {
                strafeRight(.3, targOrient);
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

            while ((robot.backDistance.getDistance(DistanceUnit.MM) > 100) && opModeIsActive()) //drivetomat
            {
                telemetry.addData("backing up", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            stopDriving();
            robot.matServoL.setPosition(grabPos);
            robot.matServoR.setPosition(freePos);
            sleep(1000); //this makes sure we don't knock the mat when we begin to go towards parking

            //Actually left towards the skybridge
            //Senses the red tape under the skybridge and tells the robot to stop

            targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while (robot.colorSensorDown.red() < 12 && opModeIsActive()) {
                strafeLeft(.3, targOrient);
                telemetry.addData("Red  ", robot.colorSensorDown.red());
                telemetry.update();
            }
            stopDriving();
        }


        if ((task == mat) && (teamcolor == red)) {
            driveForward();
            while ((robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) && opModeIsActive()) //drive to mat
            {
            }
            stopDriving();
            lastTime = runtime.milliseconds();
             //this actually makes it go right toward the center of the mat
            Orientation targOrient;
            targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while (runtime.milliseconds() < lastTime + 1000) {
                strafeLeft(.3, targOrient);
            }
            stopDriving();
            robot.matServoL.setPosition(freePos);
            robot.matServoR.setPosition(grabPos);
            sleep(1000); //We can edit this delay based on it we need more time or not

            driveBackwards();

            while ((robot.backDistance.getDistance(DistanceUnit.MM) > 100) && opModeIsActive()) //drivetomat
            {
                telemetry.addData("backing up", "Back Distance: " + robot.backDistance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            stopDriving();
            robot.matServoL.setPosition(grabPos);
            robot.matServoR.setPosition(freePos);
            sleep(1000); //this makes sure we don't knock the mat when we begin to go towards parking

            rotateR(-10.0, 0.3);

             //Actually left towards the skybridge
            //Senses the BLUE tape under the skybridge and tells the robot to stop
            targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while (robot.colorSensorDown.red() < 30 && opModeIsActive()) {
                strafeRight(.3, targOrient);
                telemetry.addData("Red  ", robot.colorSensorDown.red());
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

    //Drive Backwards - Used for starting the game
    void driveBackwardsSlow() {
        robot.frontLeftMotor.setPower(-0.3);
        robot.frontRightMotor.setPower(0.3);
        robot.backLeftMotor.setPower(-0.3);
        robot.backRightMotor.setPower(0.3);
    }


    //Drive Forwards - Towards where the Backsensor is facing
    void driveForward() {
        robot.frontLeftMotor.setPower(0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.backRightMotor.setPower(-0.5);
        robot.frontRightMotor.setPower(-0.5);
    }

    //Drive Forwards - Towards where the Backsensor is facing
    void driveForwardSlow() {
        robot.frontLeftMotor.setPower(SLOW_SPEED);
        robot.backLeftMotor.setPower(SLOW_SPEED);
        robot.backRightMotor.setPower(-1 * SLOW_SPEED);
        robot.frontRightMotor.setPower(-1 * SLOW_SPEED);
    }

    //Strafe Left - (used to strafe towards the center line for parking)
    void strafeLeft(double pwr, Orientation target) {  //added int pwr to reduce initial power
        //error stuff here
        Orientation currOrient;
        double rScale = 0.5;
        currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
        double targAng = target.angleUnit.DEGREES.normalize(target.firstAngle);
        double error = targAng-currAng;

        double r = -(error)/180*(pwr);

/*
        if(error<-10){
            pwr-=.3;
        }
        else if (error>10){
            pwr+=.3;
        }
*/

        if ((r < .07) && (r > 0)) {
            r = .07;
        } else if ((r > -.07) && (r < 0)) {
            r = -.07;
        }


        telemetry.addData("pwr:>", pwr);
        telemetry.addData("error:>", r);
        telemetry.addData("r:>", r);
        telemetry.update();

        robot.frontLeftMotor.setPower(pwr+r);
        robot.backLeftMotor.setPower(-pwr+r); //Changing the order in which the wheels start
        robot.backRightMotor.setPower(-pwr+r);
        robot.frontRightMotor.setPower(pwr+r);

    }

    void strafeRight(double pwr, Orientation target) {  //added int pwr to reduce initial power
        //error stuff here
        Orientation currOrient;
        double rScale = 0.5;
        currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
        double targAng = target.angleUnit.DEGREES.normalize(target.firstAngle);
        double error = targAng-currAng;
        double r = -(error)/180*(pwr);
  /*      if(error<-1){
            pwr-=.2;
        }
        else if (error>1){
            pwr+=.2;
        }
*/
        if ((r < .07) && (r > 0)) {
            r = .07;
        } else if ((r > -.07) && (r < 0)) {
            r = -.07;
        }

       /* telemetry.addData("pwr:>", pwr);
        telemetry.addData("error:>", r);
        telemetry.addData("r:>", r);
        telemetry.update();*/

        robot.frontLeftMotor.setPower(-pwr+r);
        robot.backLeftMotor.setPower(pwr+r); //Changing the order in which the wheels start
        robot.backRightMotor.setPower(pwr+r);
        robot.frontRightMotor.setPower(-pwr+r);
    }


    void outAndBackRed() {
        driveForwardSlow(); //Out from the parking tape under the skybridge
        sleep(1000);
        stopDriving();
        raiseClaw();
        driveForwardSlow();
        sleep(2000);
        stopDriving();
        openClaw(); //Claw servo in the open position
        sleep(300);
        stopDriving();
        driveBackwardsSlow();  //Back to the parking tape under the skybridge
        sleep(1000);
        stopDriving();
        lowerClaw();
        sleep(250);
        driveBackwardsSlow();
       /* sleep(500);
        stopDriving();
        lowerClaw();
        sleep(1000);
        driveBackwardsSlow();*/

        //Stop at the red tape
        while (robot.colorSensorDown.red() < REDTAPE && opModeIsActive()) {

        }
        stopDriving();

    }

    void outAndBackBlue() {
        driveForwardSlow(); //Out from the parking tape under the skybridge
        sleep(1000);
        stopDriving();
        raiseClaw();
        driveForwardSlow();
        sleep(2000);
        stopDriving();
        openClaw(); //Claw servo in the open position
        sleep(300);
        stopDriving();
        driveBackwardsSlow();  //Back to the parking tape under the skybridge
        sleep(1000);
        stopDriving();
        lowerClaw();
        sleep(250);
        driveBackwardsSlow();
        //Stop at the blue tape
        while (robot.colorSensorDown.blue() < BLUETAPE && opModeIsActive()) {

        }
        stopDriving();

    }

    void scan(int color) {
        boolean strafeRightBoole;
        boolean bothYellow = true;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        relativeLayout.post(new Runnable() {
            public void run() {
                robot.pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                robot.blinkinLedDriver.setPattern(robot.pattern);
            }
        });

        if (color == 2) {
            strafeRightBoole = true;

        }
        else{
            strafeRightBoole = false;
        }
        Orientation targOrient;
        targOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", "heading: " + targOrient);
        telemetry.update();
        ///ALL STRAFES ARE INVERTED IN AUTONOMOUS
        //STRAFE RIGHT IN THE AUTONOMOUS CODE IS STRAFE LEFT IN REAL LIFE
        //sorry for the all caps, it's just important
        //-Love, Graham
        //sleep(100);
        //We may need to change the alpha values to get consistent readings
        while ((bothYellow == true) && opModeIsActive()) {

            if(color == 2){
                strafeRight(STRAFE_SPEED, targOrient);
            }
            else if(color == 1){
                strafeLeft(STRAFE_SPEED, targOrient);
            }
            int skyStoneThreshold = 100;
            if ((robot.colorSensorL.alpha() > skyStoneThreshold) && (robot.colorSensorR.alpha() > skyStoneThreshold)) {
                bothYellow = true;
                //both are yellow or air
            }
            //The next two are just extra cases if it isn't reading properly
            if ((robot.colorSensorL.alpha() < skyStoneThreshold) && (robot.colorSensorR.alpha() > skyStoneThreshold)) {
                bothYellow = true;
                //left is black and right is yellow or air
            }
            if ((robot.colorSensorL.alpha() > skyStoneThreshold) && (robot.colorSensorR.alpha() < skyStoneThreshold)) {
                bothYellow = true;
                //left is yellow or air and right is black
            }

            // If it's black then bothYellow is false
            if ((robot.colorSensorL.alpha() < skyStoneThreshold) && (robot.colorSensorR.alpha() < skyStoneThreshold)) {
                sleep(30);
                bothYellow = false;

            }
            telemetry.addData("leftVal = ", "leftVal = " + robot.colorSensorL.alpha());
           telemetry.addData("rightVal = ", "rightVal = " + robot.colorSensorR.alpha());
            //telemetry.addData("bothYellowVal: ", "Yellow State: " + bothYellow);
            telemetry.update();
        }


      //  telemetry.addData("SICK", "I SEE A SKYSTONE");
        telemetry.update();
        relativeLayout.post(new Runnable() {
            public void run() {
                robot.pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;  //used to be green but want white for blue tape
                robot.blinkinLedDriver.setPattern(robot.pattern);
            }
        });

    }


    void raiseClaw() {
        robot.liftMotor.setPower(-1);
        while ((robot.liftMotor.getCurrentPosition() > -2500) && opModeIsActive()) {
            telemetry.addData("raiseClaw pos: ", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0.0);
    }

    void lowerClaw() {
        robot.liftMotor.setPower(1);
        while ((robot.liftMotor.getCurrentPosition() < 0) && opModeIsActive()) {
            telemetry.addData("lowerClaw pos: ", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0.0);
    }

    void positionRobot() {
        driveForwardSlow();
        //TODO: Get a more accurate distance
        while ((robot.frontDistance.getDistance(DistanceUnit.MM) > 140) && opModeIsActive()) {
            telemetry.addData("positionRobot  dist(mm): ", robot.frontDistance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

    }

    void grabPrep() {
        driveBackwardsSlow();
        while ((robot.frontDistance.getDistance(DistanceUnit.MM) < 200) && opModeIsActive()) {
            telemetry.addData("driveBackwards  dist(mm): ", robot.backDistance.getDistance(DistanceUnit.MM));
            telemetry.update();
            sleep(10);
        }
        stopDriving();

        lowerClaw();
        sleep(500);
    }

    void grabStone() {
        stopDriving();
        driveForwardSlow();
        while ((robot.backDistance.getDistance(DistanceUnit.MM) < 700) && opModeIsActive()) {
            sleep(10);
        }
        telemetry.addData("I stop", "I have entered the Grab Phase");
        telemetry.update();
        lowerClaw();
        sleep(500);
        closeClaw();
        sleep(500);

        driveBackwardsSlow();
        while ((robot.backDistance.getDistance(DistanceUnit.MM) > 720) && opModeIsActive()) {
            sleep(10);
        }
        stopDriving();

    }

    void openClaw() {
        robot.clawServo.setPosition(0);
    }

    void closeClaw() {
        robot.clawServo.setPosition(1);
    }


    void parkStonesRed() {
        driveForwardSlow(); //Back to the parking tape under the skybridge
        //Stop at the red tape
        while (robot.colorSensorDown.red() < REDTAPE && opModeIsActive()) {
            sleep(10);
            telemetry.addData("parking Red  ", robot.colorSensorDown.red());
            telemetry.addData("parking Alpha  ", robot.colorSensorDown.alpha());
            telemetry.update();
        }
        stopDriving();
        sleep(100);
    }


    void parkStonesBlue() {
        driveForwardSlow();  //Back to the parking tape under the skybridge
        //Stop at the blue tape
        //TODO create a blue version of the red check
        while (robot.colorSensorDown.blue() < BLUETAPE && opModeIsActive()) {
            sleep(10);
            telemetry.addData("parking Blue  ", robot.colorSensorDown.red());
            telemetry.addData("parking Alpha  ", robot.colorSensorDown.alpha());
            telemetry.update();
        }
        stopDriving();
        sleep(100);

    }

    void rotateR(double heading, double speed) {

        Orientation angles;
        Acceleration gravity;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("current heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();


        rotateRight(speed);
        while ((angles.angleUnit.DEGREES.normalize(angles.firstAngle) > heading) && opModeIsActive()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        stopDriving();
    }

    void rotateRight(double speed) {
        // Set power on each wheel
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

    }

    void rotateL(double heading, double speed) {

        Orientation angles;
        Acceleration gravity;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("current heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();


        rotateLeft(-speed);
        while ((angles.angleUnit.DEGREES.normalize(angles.firstAngle) < heading) && opModeIsActive()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        stopDriving();
    }

    void rotateLeft(double speed) {
        // Set power on each wheel
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

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


