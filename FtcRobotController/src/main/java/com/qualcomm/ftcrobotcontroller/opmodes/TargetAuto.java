package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Cormac on 12/3/2015.
 */
public class TargetAuto extends PushBotHardware
{
    public TargetAuto() {}
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor arm;
    DcMotor wormy;
    Servo leftServo;
    Servo rightServo;
    Servo armServo;
    GyroSensor sensorGyro;

    final static int ENCODER_CPR = 1440;
    final static double GEAR_RATIO = 0.5;
    final static double WHEEL_DIAMETER = 20.41;
    final static double DISTANCE1 = 48; //two tiles
    final static double DISTANCE2 = 12; //half tile
    final static double DISTANCE3 = 48; //two tiles
    final static double DISTANCE4 = 48; //two tiles

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS1 = DISTANCE1 / CIRCUMFERENCE;
    final static double ROTATIONS2 = DISTANCE2 / CIRCUMFERENCE;
    final static double ROTATIONS3 = DISTANCE3 / CIRCUMFERENCE;
    final static double ROTATIONS4 = DISTANCE4 / CIRCUMFERENCE;

    final static double COUNTS1 = ENCODER_CPR * ROTATIONS1 * GEAR_RATIO;
    final static double COUNTS2 = ENCODER_CPR * ROTATIONS2 * GEAR_RATIO;
    final static double COUNTS3 = ENCODER_CPR * ROTATIONS3 * GEAR_RATIO;
    final static double COUNTS4 = ENCODER_CPR * ROTATIONS4 * GEAR_RATIO;

    private int v_state = 0;
    private int xVal;
    private int yVal;
    private int zVal;
    private int heading;

    private int xVal1;
    private int xVal2;



    @Override public void start ()
    {
        super.init();
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        arm = hardwareMap.dcMotor.get("right_arm");
        wormy = hardwareMap.dcMotor.get("wormy");
        leftServo = hardwareMap.servo.get("left_servo");
        rightServo = hardwareMap.servo.get("right_servo");
        armServo = hardwareMap.servo.get("arm_servo");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        //reverse right motor so forward is forward
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        leftServo.setPosition(0);
        rightServo.setPosition(0);
        //armServo.setPosition(0);

        hardwareMap.logDevices();
        reset_encoders();
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();
    }
    @Override public void loop ()
//    public void runOpMode() throws InterruptedException
    {
        // get the x, y, and z values (rate of change of angle).
        sensorGyro.calibrate();
        xVal = yVal = zVal = 0;
        heading = 0;
        xVal = sensorGyro.rawX();
        yVal = sensorGyro.rawY();
        zVal = sensorGyro.rawZ();
        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        heading = sensorGyro.getHeading();
        telemetry.addData("1. x", String.format("%03d", xVal));
        telemetry.addData("2. y", String.format("%03d", yVal));
        telemetry.addData("3. z", String.format("%03d", zVal));
        telemetry.addData("4. h", String.format("%03d", heading));
//        while (gyro.isCalibrating()) {
//            try {
//                Thread.sleep(50);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
        switch (v_state)

        {
            case 0:
                //reset drive encoders
                reset_encoders();
                if (have_drive_encoders_reset())
                {
                    telemetry.addData("Case 0", "It worked!");
                }
                else
                {
                    telemetry.addData("Case 0", "Oh no!");
                }
                run_using_encoders();
                // Transition to the next state when this method is called again.
                v_state++;

                break;
//            case 1:
//                telemetry.addData("Case 1", "Hello, World!");
//                set_drive_power(1.0f,1.0f);
//
//                if (have_drive_encoders_reached(COUNTS1, COUNTS1))
//                {
//                    reset_encoders();
//
//                    set_drive_power(0f, 0f);
//
//                    v_state++;
//                }
//                break;
            case 1:
                telemetry.addData("Case 1", "Finally");

                leftMotor.setTargetPosition((int) 1693);
                rightMotor.setTargetPosition((int) 1693);

                leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                set_drive_power(1.0f, 1.0f);
                reset_drive_encoders();
                v_state++;
                break;

            case 2:
                telemetry.addData("Case 2", "Hello, World!");
                reset_drive_encoders();
                if (have_drive_encoders_reset())
                {
                    v_state++;
                }
                break;
            case 3:
                telemetry.addData("Case 3", "Hello, World!");
                v_state++;
                break;
            case 4:
                telemetry.addData("Case 4", "Hello, World!");
                leftMotor.setTargetPosition((int) COUNTS2);
                rightMotor.setTargetPosition((int) COUNTS2);

                leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                set_drive_power(-1.0f, 1.0f);

                v_state++;
                break;
            case 5:
                telemetry.addData("Case 5", "Hello, World!");
                leftMotor.setTargetPosition((int) COUNTS3);
                rightMotor.setTargetPosition((int) COUNTS3);

                leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                set_drive_power(1f, 1f);

                v_state++;
                break;
            case 6:
                telemetry.addData("Case 6", "Hello, World!");
                run_using_encoders();
                set_drive_power(0f, 0f);
                v_state++;
                break;
            case 7:
                run_using_encoders();
                set_drive_power(0f, 0f);
                v_state++;
                break;
        }
        telemetry.addData("4. x", String.format("%03d", xVal));
        telemetry.addData("5. y", String.format("%03d", yVal));
        telemetry.addData("6. z", String.format("%03d", zVal));
        telemetry.addData("7. h", String.format("%03d", heading));
        telemetry.addData("Status", "" + v_state);
        if (v_state >= 7)
        {
            telemetry.addData("Status", "Done");
        }
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();
    }
    public void update_telemetry ()

    {
        if (a_warning_generated()) {
            set_first_message(a_warning_message());
        }
        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData
                ("01"
                        , "Left Drive: "
                                + a_left_drive_power()
                                + ", "
                                + a_left_encoder_count()
                );
        telemetry.addData
                ("02"
                        , "Right Drive: "
                                + a_right_drive_power()
                                + ", "
                                + a_right_encoder_count()
                );
        telemetry.addData("1. x", String.format("%03d", xVal));
        telemetry.addData("2. y", String.format("%03d", yVal));
        telemetry.addData("3. z", String.format("%03d", zVal));
        telemetry.addData("4. h", String.format("%03d", heading));

    }
    public void update_gamepad_telemetry ()

    {
        //
        // Send telemetry data concerning gamepads to the driver station.
        //
        telemetry.addData("03", "GP1 Y: " + -gamepad1.left_stick_y);
        telemetry.addData("04", "GP1 X: " + -gamepad1.right_stick_y);
    }
    public void reset_encoders () {
        reset_left_drive_encoder();
        reset_right_drive_encoder();
//        this.leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        this.rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//
//        this.leftMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        this.rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }
    public void set_first_message (String p_message)

    {
        telemetry.addData("00", p_message);

    }

}

