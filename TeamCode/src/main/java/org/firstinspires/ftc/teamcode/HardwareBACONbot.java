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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a the BACONbot robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * Motor channel:   Front Left drive motor:       "FL"
 * Motor channel:  Front Right  drive motor:      "FR"
 * Motor channel:   Back Left drive motor:        "BL"
 * Motor channel:  Back Right  drive motor:       "BR"
 *
 */
public class HardwareBACONbot
{
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor   = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor  backLeftMotor    = null;
    public DcMotor  backRightMotor   = null;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBACONbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("FL"); // 0 - motor port
        frontRightMotor = hwMap.dcMotor.get("FR"); // 1
        backLeftMotor   = hwMap.dcMotor.get("BL"); // 2
        backRightMotor  = hwMap.dcMotor.get("BR"); // 3

        // BACONbot uses AndyMark NeverRest Motors
        // This code assumes that the motors turns counterclockwise,
        //     looking from the back of the motor down the shaft,
        //     when positive power is applied

        //  *****if the above assumption is incorrect uncomment these lines
        //frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //
        // Holonomic drive will make using encoders challenging as straffing is
        //    a design expectation
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
 }

