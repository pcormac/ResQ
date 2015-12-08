package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Cormac on 12/3/2015.
 */
public class BasicAuto extends PushBotTelemetry
{
    public BasicAuto () {}
    DcMotor leftMotor;
    DcMotor rightMotor;

    int v_state = 0;

    @Override public void init ()
    {
        super.init();
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        reset_encoders();
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();

    }
    static int[] l_times = new int [3];

    @Override public void loop()
    {

        reset_encoders();

        switch (v_state)
        {
            case 0:
                //reset drive encoders
                reset_encoders();

                // Transition to the next state when this method is called again.
                //
                l_times[0] = 0;
                l_times[1] = 0;
                l_times[2] = 0;
                v_state++;

                break;
            case 1:
                run_using_encoders();

                set_drive_power(1.0f,1.0f);

                if (have_drive_encoders_reached(1693, 1693))
                {
                    reset_encoders();

                    set_drive_power(0f, 0f);

                    v_state++;
                }
                break;
            case 2:
                if (have_drive_encoders_reset())
                {
                    v_state++;
                }
                else
                {
                    l_times[0]++;
                }
                break;
            case 3:
                run_using_encoders();
                set_drive_power(-1f, 1f);
                if (have_drive_encoders_reached(423, 423))
                {
                    set_drive_power (0.0f, 0.0f);
                    reset_encoders();
                    v_state++;
                }
                break;
            case 4:
                run_using_encoders();
                set_drive_power(1f, 1f);
                if (have_drive_encoders_reached(1693, 1693))
                {
                    reset_encoders();

                    set_drive_power(0f, 0f);

                    v_state++;
                }
                break;
            case 5:
                run_using_encoders();
                set_drive_power(0.875f, 1f);
                if (have_drive_encoders_reached(1693, 1693))
                {
                    reset_encoders();

                    set_drive_power(0f, 0f);

                    v_state++;
                }
                break;

        }
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();
    }
    public void reset_encoders ()
    {
        reset_left_drive_encoder();
        reset_right_drive_encoder();
//        this.leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        this.rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//
//        this.leftMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        this.rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }

}

