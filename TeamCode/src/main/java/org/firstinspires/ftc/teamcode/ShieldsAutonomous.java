/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// This OpMode is just Shields messing around with holonomic drive and functions
// The OpMode utilizes the BACONbot hardware definition defined in HardwareBACONbot
// =================================
// TO DO:
// - refine movement with gyros
// =================================
@Autonomous(name = "Shields and Arleth: Autonomous", group = "Shields")

public class ShieldsAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBACONbot        robot = new HardwareBACONbot();   // Use a BACONbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    int robotHeading = 0;              // Gyro integrated heading
    int targetHeading = 0;
    int headingError = 0;
    double rScale = 0.5;
    @Override
    //@Disable
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
    // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        /*robot.gyro1.calibrate();

        // make sure the gyro is calibrated.
        while (robot.gyro1.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }
*/
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        // Send telemetry message to signify robot waiting;
        //telemetry.addData(">", "Greetings, human.");
        //telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // == My fancy Autonomous code ==
        // put the sensor up
        //robot.opticalServo.setPosition(0);
        // go toward the wall until you see the wall
        headingPowerTime(225, 0.4, 4, 0.02);
        // put the sensor down
        //robot.opticalServo.setPosition(.53);
        //sleep(200);
        // strafe to the white line
        headingPowerTime(180, 0.4, 4, 0.04);
        // == End fancy autonomous code ==
        /*
        // do a little dance
        headingPowerTime(0, 1, 1, 0);
        // make a little love
        headingPowerTime(45, 1, 1, 0);
        // get down tonight
        headingPowerTime(90, 1, 1, 0);
        // get down get down
        headingPowerTime(135, 1, 1, 0);
        // do a little dance
        headingPowerTime(180, 1, 1, 0);
        // make a little love
        headingPowerTime(225, 1, 1, 0);
        // get down tonight
        headingPowerTime(270, 1, 1, 0);
        // get down get down
        headingPowerTime(315, 1, 1, 0);

        */
        /*
        // testing out servo positions
        if (gamepad1.x)
            robot.opticalServo.setPosition(0);
        if (gamepad1.y)
            robot.opticalServo.setPosition(.33);
        if (gamepad1.b)
            robot.opticalServo.setPosition(.66);
        if (gamepad1.a)
            robot.opticalServo.setPosition(1);

        while (opModeIsActive()) {
            telemetry.addData("Raw",    robot.odsSensor1.getRawLightDetected());
            telemetry.addData("Normal", robot.odsSensor1.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        */
    }

    public void goForward() {
        robot.frontLeftMotor.setPower(-.5);
        robot.frontRightMotor.setPower(.5);
        robot.backLeftMotor.setPower(-.5);
        robot.backRightMotor.setPower(.5);
    }
    public void wheelsOff() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
    public void headingPowerTime(int heading, double power, int time, double lightVal) {
        double headingRads = Math.toRadians(heading);
        double x = power*Math.cos(headingRads);
        double y = power*Math.sin(headingRads);
        //robotHeading = robot.gyro1.getHeading();
        headingError = robotHeading - targetHeading;
        double r = (headingError - 180)/180*rScale;
        // ensure minimal power to move robot
        if ((r < .07) && (r > 0)) {
            r = .07;
        } else if ((r > -.07) && (r < 0)) {
            r = -.07;
        }
        telemetry.addData("error:>", r);
        telemetry.addData("r:>", r);
        telemetry.update();
        //double r = 0;
        setWheelPower(x, y, r);
        runtime.reset();
        // get stuck in a while loop for the proper amount of time
        while (opModeIsActive() && (runtime.seconds() < time)) {
            if(lightVal > 0) {
                //double light = robot.OpticalDistanceSensor.getLightDetected();
                //telemetry.addData("light", light);
                telemetry.update();
               // if (light > lightVal)
                    break;
            }
        }
        wheelsOff();
    }
    public void setWheelPower(double x, double y, double r) {
        // calculate the power for each wheel (math from here: https://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0)
        double frontLeft  = -y - x + r;
        double frontRight = +y - x + r;
        double backLeft   = -y + x + r;
        double backRight  = +y + x + r;

        // Normalize the values so none exceeds +/- 1.0
        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
        if (max > 1.0) {
            frontLeft  = frontLeft / max;
            frontRight = frontRight / max;
            backLeft   = backLeft / max;
            backRight  = backRight / max;
        }
        // Set power on each wheel
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
        robot.backLeftMotor.setPower(backLeft);
        robot.backRightMotor.setPower(backRight);
    }
}
