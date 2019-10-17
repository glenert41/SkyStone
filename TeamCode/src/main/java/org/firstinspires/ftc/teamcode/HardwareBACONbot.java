package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
 *
 */
public class HardwareBACONbot
{
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor   = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor  backLeftMotor    = null;
    public DcMotor  backRightMotor   = null;
    public DcMotor  shooterMotor   = null;

    public ColorSensor colorSensor1  = null;
    public boolean bLedOn = true;    // bLedOn represents the state of the LED.

    public ModernRoboticsI2cGyro gyro1;
    public OpticalDistanceSensor odsSensor1;
    public TouchSensor touchSensor1;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBACONbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

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
//        shooterMotor.setPower(0);

        // Set all motors to run without encoders.
        // Holonomic drive will make using encoders challenging as straffing is
        //    a design expectation
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//       shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // get a reference to our ColorSensor object.
//        colorSensor1 = hwMap.colorSensor.get("color1");
        // Set the LED in the beginning
//        colorSensor1.enableLed(bLedOn);

        // get a reference to a Modern Robotics GyroSensor object.
//        gyro1 = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro1");

        // get a reference to our Light Sensor object.
//        odsSensor1 = hwMap.opticalDistanceSensor.get("optical1");
/*
        // get a reference to our Light Sensor object.
        touchSensor1 = hwMap.touchSensor.get("touch1");
  */
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

