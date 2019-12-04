/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BACON: Mechanum", group="Opmode")
//@Disabled
public class BACONbotMechanum extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareBACONbot robot = new HardwareBACONbot();   // Use a BACONbot's hardware

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double meetDistance = 790; //Distance from wall to the Blocks/Mat (CM From Wall (BackSensor))
        double leftMeetDistance = 10;

        double grabPos = 0;  //change these later (written 12-3-19)
        double freePos = 1;  //change these later  (written 12-3-19)








        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // In this mode the Left stick moves the robot in the direction pointed to by x,y
            //              the Right stick x controls rotation; right (positive) rotates clockwise

            double x;
            double y;
            double r;
            double frontLeft;
            double frontRight;
            double backLeft;
            double backRight;
            double max;

            // Get x and y values from left joystick. (With the Logitech 310 the joystick y goes negative when pushed forwards, so negate it)
            // Get the x value of the right joystick.



            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            r = gamepad1.right_stick_x;

            // do not let rotation dominate movement
            r = r / 4;

            // calculate the power for each wheel

            frontLeft = +y - x + r;
            backLeft = +y + x + r;

            frontRight = -y - x + r;
            backRight = -y + x + r;

            // Normalize the values so none exceeds +/- 1.0
            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }

            // Set power on each wheel
            robot.frontLeftMotor.setPower(frontLeft);
            robot.frontRightMotor.setPower(frontRight);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);


            // Show wheel power to driver
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("front left", "%.2f", frontLeft);
            telemetry.addData("front right", "%.2f", frontRight);
            telemetry.addData("back left", "%.2f", backLeft);
            telemetry.addData("back right", "%.2f", backRight);
            telemetry.addData("back distance--", String.format("%.01f mm", robot.backDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("left distance--", String.format("%.01f mm", robot.leftDistance.getDistance(DistanceUnit.MM))); //Added this one
            telemetry.addData("colorSensor--", String.format("%.01f mm", robot.distanceSensorL.getDistance(DistanceUnit.MM))); //Added this one


            telemetry.update();



//550
            //BUILD SIDE AUTO - IN PROGRESS (Tuning)---------------------------------
            if (gamepad1.x) {  //We changed it from an a to an x so that it would start every time we pressed start+a to initialize the controller
                while (robot.backDistance.getDistance(DistanceUnit.MM) < 550) { //**Changed the number here- not sure its quite perfect yet but this is the best we have gotten
                    driveBackwards();
                }

                stopDriving();

                /* Sample code for taking yellow/black readings */
                // TODO: replace hardwareMap with robot after adding color sensor to hardwareMap
                ColorSensor sensorColorRangeAsREVColorRangeSensor;
                sensorColorRangeAsREVColorRangeSensor = hardwareMap.colorSensor.get("sensorColorRangeAsREVColorRangeSensor");

                ColorSensor bottomColorSensor;
                bottomColorSensor = hardwareMap.colorSensor.get("bCS");


                if(ColorBot.isBlack(sensorColorRangeAsREVColorRangeSensor)) {
                    telemetry.addData("Object is Black",sensorColorRangeAsREVColorRangeSensor.red());

                    //Detects the Skystone

                    while (robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance+20) {
                        driveBackwards();
                    }

                    stopDriving();
                    robot.clawServo.setPosition(grabPos); //sets the servo to Grab Position

                    while (robot.backDistance.getDistance(DistanceUnit.MM) > meetDistance-20){
                        driveForward();
                    }

                    //make the isNothing in the ColorBot thing
/******

while(ColorBot.isNOTHING(bottomColorSensor)) {
                    strafeLeft(3);

                    }
*****/

                    stopDriving();
                    outAndBack();



                }

                else if (ColorBot.isYellow(sensorColorRangeAsREVColorRangeSensor)) {
                    telemetry.addData("Object is Yellow",sensorColorRangeAsREVColorRangeSensor.red());

                    robot.clawServo.setPosition(freePos);

                }
                else
                {
                    telemetry.addData("Object is neither Black nor Yellow",sensorColorRangeAsREVColorRangeSensor.red());

                    robot.clawServo.setPosition(freePos);

                }
                //**This next part is commented out for the moment, but we might want to add it back in (but it works without it)
              /* while (robot.backDistance.getDistance(DistanceUnit.MM) > 25){
                   driveForward();
               }
               stopDriving();
           }  */
                telemetry.update();
                while ((robot.leftDistance.getDistance(DistanceUnit.MM) > 40)&&(ColorBot.isYellow(sensorColorRangeAsREVColorRangeSensor))) {
                    strafeLeft(.3);


                }

                stopDriving();
            }














        if (gamepad1.b) {     //When we press B, strafe towards the center of the tape - The Press B part will be changed later

                strafeLeft(.3);
            }
            //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
/*
            //Mat Servo Tuning
            double open_Pos = 0;          //CHANGE LATER
            double clamp_Pos = 1;         //CHANGE LATER
            double matServoPosition;

            if(gamepad1.left_bumper){

                matServoPosition = clamp_Pos;
            }
            else{
                matServoPosition = open_Pos;
            }
            robot.matServo.setPosition(matServoPosition);
*/

        }







        //Loading Zone Side Start -- Starting -- Mekhi and Alden

        if(gamepad1.y) {

            while (robot.backDistance.getDistance(DistanceUnit.MM) < meetDistance) {
                driveBackwards();
            }

            stopDriving();


            while (robot.leftDistance.getDistance(DistanceUnit.MM) < leftMeetDistance) {
                strafeLeft(.3);
            }

            //PseudoCode
            /*
            While the distance (using a new sensor) is greater than a certain distance, drive to the left.
            There is a function strafeLeft();
             */

        }









    }





    // Functions --------------------------------------


    //Driving Functions

    //Stop Driving - Kill power to all the motors
    void stopDriving(){

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);


    }

    //Drive Backwards - Used for starting the game
    void driveBackwards(){
        robot.frontLeftMotor.setPower(-0.5);
        robot.frontRightMotor.setPower(0.5);
        robot.backLeftMotor.setPower(-0.5);
        robot.backRightMotor.setPower(0.5);

    }

    //Drive Forwards - Towards where the Backsensor is facing
    void driveForward(){
        robot.frontLeftMotor.setPower(0.5);
        robot.frontRightMotor.setPower(-0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.backRightMotor.setPower(-0.5);
    }

    //Strafe Left - (used to strafe towards the center line for parking)
    void strafeLeft(double pwr){  //added int pwr to reduce initial power
        robot.frontLeftMotor.setPower(pwr);
        robot.backRightMotor.setPower(-pwr); //Changing the order in which the wheels start
        robot.frontRightMotor.setPower(pwr);
        robot.backLeftMotor.setPower(-pwr);

    }

    void strafeRight(double pwr){  //added int pwr to reduce initial power
        robot.frontLeftMotor.setPower(-pwr);
        robot.backRightMotor.setPower(pwr); //Changing the order in which the wheels start
        robot.frontRightMotor.setPower(-pwr);
        robot.backLeftMotor.setPower(pwr);

    }

    void outAndBack(){
        strafeLeft(3);
        //TimeUnit.SECONDS.sleep(1);
        stopDriving();
        robot.clawServo.setPosition(1);        //UPDATE THIS NUMBER TO WHATEVER freePOS is
        stopDriving();
        strafeRight(3);
    }
}
