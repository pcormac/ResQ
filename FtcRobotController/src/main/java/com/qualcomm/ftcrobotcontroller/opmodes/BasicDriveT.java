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
public class BasicDriveT extends PushBotTelemetry {

    public BasicDriveT ()

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
    DcMotor arm;
    DcMotor wormy;
    DcMotor winch;
    Servo leftServo;
    Servo rightServo;
    Servo armServo;
    Servo plow;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        arm = hardwareMap.dcMotor.get("right_arm");
        wormy = hardwareMap.dcMotor.get("wormy");
        winch = hardwareMap.dcMotor.get("winch");
        leftServo = hardwareMap.servo.get("left_servo");
        rightServo = hardwareMap.servo.get("right_servo");
        armServo = hardwareMap.servo.get("arm_servo");
        plow = hardwareMap.servo.get("plow_servo");

        //reverse right motor so forward is forward
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
        winch.setDirection(DcMotor.Direction.REVERSE);
        leftServo.scaleRange(0, 1);
        rightServo.scaleRange(0, 1);

        leftServo.setPosition(0);
        rightServo.setPosition(0);
        armServo.setPosition(0);
    }

    @Override
    public void loop() {
        /*
        Gamepad 1:
            Used:
                Left stick
                Right stick
                Button X
                Button Y
            Unused:
                Button A
                Button B
                LT
                RT
                LB
                RB
        Gamepad 2:
            Used:
                Right stick
                Button Y
                Button A
                Button B
                Button X
            Unused:
                Left stick
                LT
                RT
                LB
                RB
         */
        //DRIVE
        float leftVal = -gamepad1.left_stick_y;
        float rightVal = -gamepad1.right_stick_y;
        // calc value for each motor
        float leftPower = leftVal;
        float rightPower = rightVal;
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);
        // set_drive_power (leftPower, rightPower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        //END OF DRIVE

        //worm gear
        float worm = gamepad1.right_stick_y;
        worm = Range.clip(worm, -1, 1);
        if (gamepad1.b)
        {
            wormy.setPower(1);
        }
        else if (gamepad1.a)
        {
            wormy.setPower(-1);
        }
        else {
            wormy.setPower(0);
        }
        //plow
        if (gamepad1.x)
        {
            plow.setPosition(0);
        }
        else if (gamepad1.y)
        {
            plow.setPosition(1);
        }
        /*
        if (gamepad1.a)
        {
            winch.setPower(1);
        }
        else
        {
            winch.setPower(0);
        }
        */
        // servos
        if (gamepad2.a)
        {
            leftServo.setPosition(1);
        }
        else
        {
            leftServo.setPosition(0);
        }
        if (gamepad2.b)
        {
            rightServo.setPosition(1);
        }
        else
        {
            rightServo.setPosition(0);
        }
        float armVal = gamepad2.right_stick_y;
        armVal = Range.clip(armVal, -1, 1);
        arm.setPower(armVal);
        //arm servo
        if (gamepad2.x)
        {
            armServo.setPosition(.7);
        }
        else
        {
            armServo.setPosition(0);
        }

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