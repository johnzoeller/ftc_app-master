//Copyright (c) 2015 Manta Mechanics Inc All rights reserved.

//default package statement from the ftc_app-master
//look into changing this to edu.mantamechanics or org.mantamechanics
package com.qualcomm.ftcrobotcontroller.opmodes;

//import statements for Modern Robotics hardware
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 *
 * Enables control of the robot via the gamepad
 */

public class MantaResQTeleOp extends OpMode {

    /*
     * Configuration of motors, servos, and sensors:
     * as the FLIPPER servo approaches 0, the FLIPPER position moves up (away from the floor).
     * Also, as the TURRET servo approaches 0, the TURRET opens up (drops the game element).
     */

    // TETRIX VALUES.
    final static double FLIPPER_MIN_RANGE  = 0.20;
    final static double FLIPPER_MAX_RANGE  = 0.90;
    final static double TURRET_MIN_RANGE  = 0.20;
    final static double TURRET_MAX_RANGE  = 0.7;

    // position of the FLIPPER servo.
    double flipperPosition;

    // amount to change the FLIPPER servo position.
    double flipperDelta = 0.1;

    // position of the TURRET servo
    double turretPosition;

    // amount to change the TURRET servo position by
    double turretDelta = 0.1;


    //drive motors (tank drive)
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorFlipper;

    //collector, turret, and elevator motors
    DcMotor motorCollector;
    DcMotor motorElevator;
    DcMotor motorTurret;

    //
    Servo flipper;
    Servo turret;

    /**
     * Constructor
     */
    public MantaResQTeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * MantaResQ assumes the following:
		 *   Tank Drive:
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 *   Collector and Elevator
		 *   "motor_3" is the Collector motor
		 *   "motor_4" is the Elevator motor
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the FLIPPER joint of the manipulator.
		 *    "servo_6" controls the TURRET joint of the manipulator.
		 */
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorCollector = hardwareMap.dcMotor.get("motor_3");
        motorElevator = hardwareMap.dcMotor.get("motor_4");


        flipper = hardwareMap.servo.get("servo_1");
        turret = hardwareMap.servo.get("servo_6");

        // assign the starting position of the wrist and TURRET

        flipperPosition = 0.5;
        turretPosition = 0.5;
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */

    @Override
    public void loop() {

		/*
		 * Gamepad 1 controls:
		 * - DRIVE
		 * --left stick drives in "arcade" style (forward/reverse/turn left/turn right)
		 * --throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
		 * --direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
		 *
		 * --the left and right trigger controls the FLIPPER position
		 *
		 * COLLECT AND SCORE
		 * -- the right stick turns the turret left and right
		 * -- buttons Y and A control the COLLECTOR/SCORER
		 *
		 *
		 *
		 */

        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        boolean elevatorUp = gamepad1.dpad_up;
        boolean elevatorDn = gamepad1.dpad_down;
        float collectorIntake = gamepad1.right_trigger;
        float collectorScore = gamepad1.left_trigger;


        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //elevator = Range.clip(elevator, -1, 1);
        //collectorSpeed = Range.clip(collectorSpeed, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        //elevator = (float)scaleInput(elevator);
        //collectorSpeed =  (float)scaleInput(collectorSpeed);

        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);
        //motorCollector.setPower(collectorSpeed);
        //motorElevator.setPower(elevator);

        // update the position of the FLIPPER.
        if (gamepad1.a) {
            // if the A button is pushed on gamepad1, increment the position of
            // the FLIPPER servo.
            flipperPosition += flipperDelta;
        }

        if (gamepad1.y) {
            // if the Y button is pushed on gamepad1, decrease the position of
            // the FLIPPER servo.
            flipperPosition -= flipperDelta;
        }

        // update the position of the TURRET
        //if (gamepad1.x) {
        //    turretPosition += turretDelta;
        //}

        //if (gamepad1.b) {
        //    turretPosition -= turretDelta;
        //}

        if (gamepad2.y) {
            motorCollector.setPower(0.75);
        }

        else if (gamepad2.a) {
            motorCollector.setPower(-0.75);
        }

        else {
            motorCollector.setPower(0.0);
        }

        if (gamepad2.x) {
            motorElevator.setPower(0.75);
        }

        else if (gamepad2.b) {
            motorElevator.setPower(-0.75);
        }

        else {
            motorElevator.setPower(0.0);
        }

        // clip the position values so that they never exceed their allowed range.
        //flipperPosition = Range.clip(flipperPosition, FLIPPER_MIN_RANGE, FLIPPER_MAX_RANGE);
        //turretPosition = Range.clip(turretPosition, TURRET_MIN_RANGE, TURRET_MAX_RANGE);

        // write position values to the wrist and TURRET servo
        //flipper.setPosition(flipperPosition);
        //turret.setPosition(turretPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Flipper", "flipper:  " + String.format("%.2f", flipperPosition));
        telemetry.addData("Turret", "turret:  " + String.format("%.2f", turretPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
