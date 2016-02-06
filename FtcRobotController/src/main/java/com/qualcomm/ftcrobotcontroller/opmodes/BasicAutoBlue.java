package com.qualcomm.ftcrobotcontroller.opmodes;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;

/**
 * Created by Cormac on 12/3/2015.
 */
public class BasicAutoBlue extends PushBotTelemetry
{
    public BasicAutoBlue () {}
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor arm;
    DcMotor wormy;
    Servo leftServo;
    Servo rightServo;
    Servo armServo;
    //   GyroSensor gyro;

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

    final static double COUNTS1 = ENCODER_CPR * ROTATIONS1 * GEAR_RATIO; //1693
    final static double COUNTS2 = ENCODER_CPR * ROTATIONS2 * GEAR_RATIO; //423
    final static double COUNTS3 = ENCODER_CPR * ROTATIONS3 * GEAR_RATIO; //1693
    final static double COUNTS4 = ENCODER_CPR * ROTATIONS4 * GEAR_RATIO; //1693

    static int C1 = 1693;
    static int C2 = 423;
    static int C3 = 1693;
    static int C4 = 1693;
    static int C5 = 0;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private double TARGET_ANGLE_DEGREES = 45.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;

    int v_state = 0;



    @Override public void init ()
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

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);
        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        df = new DecimalFormat("#.##");

        //reverse right motor so forward is forward
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(0);
        rightServo.setPosition(0);
        armServo.setPosition(0);

        hardwareMap.logDevices();

//        gyro.calibrate();
        //GyroSensor gyro;
        int xVal, yVal, zVal = 0;
        int heading = 0;

        leftServo.setPosition(0);
        rightServo.setPosition(0);
        armServo.setPosition(0);
        resetEncoders();

        update_telemetry(); // Update common telemetry
        //update_gamepad_telemetry();

    }
    @Override public void start() {
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }

    @Override public void loop()

//    public void runOpMode() throws InterruptedException
    {
        // get the x, y, and z values (rate of change of angle).

        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
//        heading = gyro.getHeading();
//        telemetry.addData("1. x", String.format("%03d", xVal));
//        telemetry.addData("2. y", String.format("%03d", yVal));
//        telemetry.addData("3. z", String.format("%03d", zVal));
//        telemetry.addData("4. h", String.format("%03d", heading));
//        while (gyro.isCalibrating()) {
//            Thread.sleep(50);
//        }
        resetEncoders();

        switch (v_state)
        {
            case 0:
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                try {
                    Thread.sleep(10000); //10 sec
                }
                catch (InterruptedException e){
                    e.printStackTrace();
                }
                // Transition to the next state when this method is called again.
                v_state++;

                break;
            case 1:
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                leftMotor.setTargetPosition(C1);
                rightMotor.setTargetPosition(C1);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                if (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(C1) &&
                        Math.abs(leftMotor.getCurrentPosition()) < Math.abs(C1)) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                    try {
                        telemetry.addData("Step", "1");
                        Thread.sleep(1000);
                        v_state++;
                    }
                    catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }
                break;
            case 2:
                if (navx_device.isConnected() &&
                        ( !navx_device.isCalibrating() )) {
                    if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                        if (yawPIDResult.isOnTarget()) {
                            leftMotor.setPowerFloat();
                            rightMotor.setPowerFloat();
                            telemetry.addData("Motor Output", df.format(0.00));
                        } else {
                            double output = yawPIDResult.getOutput();
                            leftMotor.setPower(output);
                            rightMotor.setPower(-output);
                            telemetry.addData("Motor Output", df.format(output) + ", " +
                                df.format(-output));
                    }
                    } else {

                        telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                }
                } else {
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    leftMotor.setTargetPosition(500);
                    rightMotor.setTargetPosition(-500);
                    leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(1);
                    rightMotor.setPower(1);

                    if (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(-500) &&
                            Math.abs(leftMotor.getCurrentPosition()) < Math.abs(500)) {
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);

                        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                        try {
                            telemetry.addData("Step", "2");
                            Thread.sleep(1000);
                            v_state++;
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
                break;
            case 3:
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                leftMotor.setTargetPosition(C2);
                rightMotor.setTargetPosition(C2);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                if (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(C2) &&
                        Math.abs(leftMotor.getCurrentPosition()) < Math.abs(C2)) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                    try {
                        telemetry.addData("Step", "2");
                        Thread.sleep(1000);
                        v_state++;
                    }
                    catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }
                break;
            case 4:
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                leftMotor.setTargetPosition(C3);
                rightMotor.setTargetPosition(C3);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                if (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(C3) &&
                        Math.abs(leftMotor.getCurrentPosition()) < Math.abs(C3)) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                    try {
                        telemetry.addData("Step", "4");
                        Thread.sleep(1000);
                        v_state++;
                    }
                    catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }
                break;
            case 5:
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                leftMotor.setTargetPosition(C4);
                rightMotor.setTargetPosition(C4);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                if (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(C4) &&
                        Math.abs(leftMotor.getCurrentPosition()) < Math.abs(C4)) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                    try {
                        telemetry.addData("Step", "5");
                        Thread.sleep(1000);
                        v_state++;
                    }
                    catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }
                break;
            case 6:
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                leftMotor.setTargetPosition(C5);
                rightMotor.setTargetPosition(C5);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                if (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(C5) &&
                        Math.abs(leftMotor.getCurrentPosition()) < Math.abs(C5)) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                    try {
                        telemetry.addData("Step", "6");
                        Thread.sleep(1000);
                        v_state++;
                    }
                    catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }
                break;
            /*case 6:
                break;
            case 7:
                break;
            case 8:
                break;
            */
        }
        update_telemetry(); // Update common telemetry
        //update_gamepad_telemetry();
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
    }
    void resetEncoders() {
        double x = 0;
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        try {
            while (rightMotor.getTargetPosition() != 0 && leftMotor.getTargetPosition() != 0) {
                telemetry.addData("Step", "resetEncoders Loop");
                telemetry.addData("Left Position", leftMotor.getCurrentPosition());
                telemetry.addData("Right Position", rightMotor.getCurrentPosition());
                Thread.sleep(10);
                x++;
            }
        }
        catch (InterruptedException e){
            // Restore the interrupted status
            Thread.currentThread().interrupt();
        }
        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
    } // End resetEncoders
    void driveToDistance(int distance) throws InterruptedException {
        double leftPosition;
        double rightPosition;
        double saveleftPosition = distance;
        double saveRightPosition = distance;

        if (distance == 0)
            return;
        leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(distance);
        leftMotor.setPower(0.75);
        rightMotor.setPower(0.75);

        while (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(distance) &&
                Math.abs(leftMotor.getCurrentPosition()) < Math.abs(distance)) {
            telemetry.addData("Step", "driveDistanceLoop");
            leftPosition = leftMotor.getCurrentPosition();
            rightPosition = rightMotor.getCurrentPosition();
            telemetry.addData("Left Position", leftPosition);
            telemetry.addData("Right Position", rightPosition);
            if ((leftPosition == saveleftPosition && Math.abs(leftPosition)+5 >= Math.abs(distance)) ||
                    (rightPosition == saveRightPosition && Math.abs(rightPosition)+5 >= Math.abs(distance))) {
                break;
            }
            saveleftPosition = leftPosition;
            saveRightPosition = rightPosition;
        } //End While

        telemetry.addData("Step", "driveDistanceExitLoop");
        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());

        resetEncoders();

    } // End driveToDistance

    //*************************************************
//  turnDistance
//*************************************************
    void turnDistance(int leftDistance, int rightDistance) throws InterruptedException {
        double leftPosition;
        double rightPosition;
        double saveleftPosition = leftDistance;
        double saveRightPosition = rightDistance;

        if (leftDistance == 0)
            return;

        leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(leftDistance);
        rightMotor.setTargetPosition(rightDistance);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        while (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(leftDistance) &&
                Math.abs(rightMotor.getCurrentPosition()) < Math.abs(rightDistance)) {
            telemetry.addData("Step", "turnDistanceLoop");
            leftPosition = leftMotor.getCurrentPosition();
            rightPosition = rightMotor.getCurrentPosition();
            telemetry.addData("Left Position", leftPosition);
            telemetry.addData("Right Position", rightPosition);
            if ((leftPosition == saveleftPosition && Math.abs(leftPosition)+5 >= Math.abs(leftDistance)) ||
                    (rightPosition == saveRightPosition && Math.abs(rightPosition)+5 >= Math.abs(rightDistance))) {
                break;
            }
            saveleftPosition = leftPosition;
            saveRightPosition = rightPosition;
        }

        telemetry.addData("Step", "turnDistanceExitLoop");
        leftPosition = leftMotor.getCurrentPosition();
        rightPosition = rightMotor.getCurrentPosition();
        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
        resetEncoders();

    } // End turnDistance
}

