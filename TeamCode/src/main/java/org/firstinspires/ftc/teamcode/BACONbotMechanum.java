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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * This OpMode provides teleop control using the left and right game controller joysticks to
 * drive a four motor four wheel holonomic robot.  The left joystick controls the direction
 * and speed of the movement.  The right joystick controls the rotation around the z axis.
 * <p>
 * The OpMode utilizes the BACONbot hardware definition defined in HardwareBACONbot
 * <p>
 * The code is based on lesson from Cody
 * https://www.youtube.com/watch?v=Pz17vCHaDIY
 * https://www.youtube.com/watch?v=20M62Xil5s4
 *
 */

@TeleOp(name="Mechanum2", group="BACONbot")
//@Disabled
public class BACONBotMechanum extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareBACONbot robot = new HardwareBACONbot();   // Use a BACONbot's hardware


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
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
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
