//Copyright (c) 2015 Manta Mechanics Inc All rights reserved.

//default package statement from the ftc_app-master
//look into changing this to edu.mantamechanics or org.mantamechanics
package com.qualcomm.ftcrobotcontroller.opmodes;

//import statements for Modern Robotics hardware
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode code
 *
 *
 */

public class MantaResQTeleOp extends OpMode {

    //declare drive motors (tank drive)
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorFlipper;

    //declare collector, turret, and elevator motors
    DcMotor motorCollector;
    DcMotor motorElevator;
    DcMotor motorTurret;

    //declare servos
    Servo flipper;
    Servo turret;

    //declare HiTechnic Motor Controller motors
    DcMotorController.DeviceMode devMode;
    DcMotorController flipperTurretController;

    // set numOpLoops to "1"
    // every 17 loops, switch to read mode to read data from the NXT device
    // The NxtDcMotorController, you need to switch into "read" mode
    // before doing a read, and into "write" mode before doing a write. This is because
    // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
    // using the USBDcMotorController, there is no need to switch, because USB can handle reads
    // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
    // This method does nothing on USB devices, but is needed on Nxt devices.
    int numOpLoops = 1;

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

        // get the legacy module (the name must match the Robot Controller
        // configuration name
        flipperTurretController = hardwareMap.dcMotorController.get("legacy_1");

        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorCollector = hardwareMap.dcMotor.get("motor_3");
        motorElevator = hardwareMap.dcMotor.get("motor_4");
        motorFlipper = hardwareMap.dcMotor.get("motor_5");
        motorTurret = hardwareMap.dcMotor.get("motor_6");

        flipper = hardwareMap.servo.get("servo_1");
        turret = hardwareMap.servo.get("servo_6");

        // assign the starting position of the wrist and TURRET
        //flipperPosition = 0.5;
        //turretPosition = 0.5;
    }

    /*
   * Code that runs repeatedly when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
   */
    @Override
    public void init_loop() {

        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

        //motorFlipper.setDirection(DcMotor.Direction.REVERSE);
        //motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        motorFlipper.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorTurret.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

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
		 * -- left stick drives in "arcade" style (forward/reverse/turn left/turn right)
		 * -- throttle: left_stick_y ranges from -1 to 1 (-1 is full up, and 1 is full down)
		 * -- direction: left_stick_x ranges from -1 to 1 (-1 is full left and 1 is full right)
		 *
		 * -- the right trigger raises the FLIPPER
		 * -- the right bumper lowers the FLIPPER
		 *
		 *
		 * COLLECT AND SCORE
		 * -- the left trigger scores (reverses the collector)
		 * -- the left bumper collects (runs the collector motor)
		 * -- the d-pad controls the elevator (up for up, down for down)
		 * -- the d-pad controls the turret (left and right)
		 *
		 *
		 *
		 */

        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //flipperUp = Range.clip(flipperUp, -1, 1);
        //collectorScore = Range.clip(collectorScore, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // write the values to the drive motors
        motorRight.setPower(right);
        motorLeft.setPower(left);

        // gamepad control of COLLECTOR
        if (gamepad1.left_bumper) {
            motorCollector.setPower(0.60);
        }
        else if (gamepad1.left_trigger == 1) {
            motorCollector.setPower(-0.60);
        }
        else {
            motorCollector.setPower(0.0);
        }

        // gamepad control of ELEVATOR
        if (gamepad1.dpad_down) {
            motorElevator.setPower(-0.50);
        }
        else if (gamepad1.dpad_up) {
            motorElevator.setPower(0.50);
        }
        else {
            motorElevator.setPower(0.0);
        }






        /*
         * code for the legacy module which starts in "write" mode
         * which is set in the init_loop method
         *
         */

        if (allowedToWrite()) {

            // gamepad control of FLIPPER
            if (gamepad1.right_bumper) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorFlipper.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorFlipper.setPower(0.6);
            }
            else if (gamepad1.right_trigger == 1) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorFlipper.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorFlipper.setPower(-0.4);
            }
            else {
                motorFlipper.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorFlipper.setPower(0.0);
            }

            // gamepad control of TURRET
            if (gamepad1.dpad_right) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorTurret.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorTurret.setPower(-0.60);
            }
            else if (gamepad1.dpad_left) {
                motorTurret.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorTurret.setPower(0.60);
            }
            else {
                motorTurret.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorTurret.setPower(0.0);
            }
        }



        // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
        // It takes time for the hardware to switch, so you can't switch modes within one loop of the
        // op mode. Every 17th loop, this op mode switches to READ_ONLY mode, and gets the current power.
        if (numOpLoops % 17 == 0){
            // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
            // before doing a read, and into "write" mode before doing a write. This is because
            // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
            // using the USBDcMotorController, there is no need to switch, because USB can handle reads
            // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
            // This method does nothing on USB devices, but is needed on Nxt devices.
            flipperTurretController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        }

        // Every 17 loops, switch to read mode so we can read data from the NXT device.
        // Only necessary on NXT devices.
        if (flipperTurretController.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {

            // Update the reads after some loops, when the command has successfully propagated through.


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("Flipper", "flipper:  " + String.format("%.2f", flipperPosition));
        //telemetry.addData("Turret", "turret:  " + String.format("%.2f", turretPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

            // Only needed on Nxt devices, but not on USB devices
            flipperTurretController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

            // Reset the loop
            numOpLoops = 0;
        }

        // Update the current devMode
        devMode = flipperTurretController.getMotorControllerDeviceMode();
        numOpLoops++;
    }

    // If the device is in either of these two modes, the op mode is allowed to write to the HW.
    private boolean allowedToWrite(){

        return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);

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
        double[] scaleArray = {
                0.00,
                0.05,
                0.09,
                0.10,
                0.12,
                0.15,
                0.18,
                0.24,
                0.30,
                0.36,
                0.43,
                0.50,
                0.60,
                0.72,
                0.85,
                1.00,
                1.00};

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
