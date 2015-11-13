package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Created by Cormac on 11/2/2015.
 */
public class BasicDrive extends PushBotTelemetry {

    public BasicDrive ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction. //su

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // BasicDrive

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        //reverse right motor so forward is forward
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float xValue = gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;

        //calc value for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        //clip range aka set max and min
        //leftPower = Range.clip(leftPower, -1, 1);
        //rightPower = Range.clip(rightPower, -1, 1);

        //set power
        //float l_left_drive_power = scale_motor_power (-gamepad1.left_stick_y);
        //float l_right_drive_power = scale_motor_power (-gamepad1.right_stick_y);

        //set_drive_power (leftPower, rightPower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();
    }
}

//
//y = 1, x = 0
//        leftMotor = 1, rightMotor = 1
//
//y = -1, x = 0
//        leftMotor = -1, rightMotor = -1
//
//y = 0, x = -1
//        leftMotor = 0, rightMotor = 1
//
//y = 0, x = 1
//        leftMotor = 1, rightMotor = 0
//
//y = .5, x = .5
//        leftMotor = .75, rightMotor = .25
//
//y = -.5, x = -.5
//        leftMotor = .25, rightMotor = .75