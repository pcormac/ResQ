package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Cormac on 12/3/2015.
 */
public class Brandons  extends PushBotTelemetry
{
    public Brandons ()
    {

    }
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override public void init ()
    {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
    }
    @Override public void start ()
    {
        super.start();
        leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }
    @Override public void loop ()
    {
        set_drive_power (1f, 1f);
        if (have_drive_encoders_reached (11520, 11520))
        {
            leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
            rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            set_drive_power (0.0f, 0.0f);
        }

        //drive forward 2 tiles, 2500 ticks?
        //use gyro to determine 45 degrees
        leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        set_drive_power (-1f, 1f);
        if (have_drive_encoders_reached (2880, 2880))
        {
            leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
            rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            set_drive_power (0.0f, 0.0f);
        }


        leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        set_drive_power (1f, 1f);
        if (have_drive_encoders_reached (11520, 11520))
        {
            reset_drive_encoders ();
            set_drive_power (0.0f, 0.0f);
        }

        leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        set_drive_power(7 / 8f, 1f);
        leftMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        if (have_drive_encoders_reached (5760, 5760))
        {
            leftMotor.setChannelMode(DcMotorController.RunMode .RESET_ENCODERS);
            rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            set_drive_power (0.0f, 0.0f);
        }



    }
}

