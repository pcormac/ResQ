package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;
import com.qualcomm.ftcrobotcontroller.opmodes.navXDriveStraightPIDLoopOp;

/**
 * Created by Cormac on 12/3/2015.
 */
public class CompRed extends PushBotTelemetry
{
    public CompRed () {}
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor arm;
    DcMotor wormy;
    Servo leftServo;
    Servo rightServo;
    Servo armServo;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private ElapsedTime runtime = new ElapsedTime();
    private double TARGET_ANGLE_DEGREES = 45.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private double TOTAL_RUN_TIME_SECONDS = 0;

    private boolean calibration_complete = false;

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;

    int v_state = 0;

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }


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

        leftServo.setPosition(0);
        rightServo.setPosition(0);
        armServo.setPosition(0);
        resetEncoders();

        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();

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
                    Thread.sleep(8000); //8 sec
                }
                catch (InterruptedException e){
                    e.printStackTrace();
                }
                // Transition to the next state when this method is called again.
                v_state++;
                break;
            case 1:
                TOTAL_RUN_TIME_SECONDS = 4;
                drive_straight_lin();
                v_state++;
                break;
            case 2:
                TARGET_ANGLE_DEGREES = -45.0;
                turn_to_angle();
                v_state++;
                break;
            case 3:
                TOTAL_RUN_TIME_SECONDS = 4;
                drive_straight_lin();
                v_state++;
                break;
            case 4:
                TOTAL_RUN_TIME_SECONDS = 4;
                drive_straight_lin();
                v_state++;
                break;
            case 5:
                TOTAL_RUN_TIME_SECONDS = 4;
                drive_straight_lin();
                v_state++;
                break;
            case 6:
                TOTAL_RUN_TIME_SECONDS = 4;
                drive_straight_lin();
                v_state++;
                break;
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
    }
    void resetEncoders() {
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        try {
            while (rightMotor.getTargetPosition() != 0 && leftMotor.getTargetPosition() != 0) {
                telemetry.addData("Step", "resetEncoders Loop");
                telemetry.addData("Left Position", leftMotor.getCurrentPosition());
                telemetry.addData("Right Position", rightMotor.getCurrentPosition());
                Thread.sleep(10);
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

    }
    public void drive_straight(){
        TARGET_ANGLE_DEGREES = 0;
        if ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if ( calibration_complete ) {
                navx_device.zeroYaw();
            } else {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        } else {
            /* Wait for new Yaw PID output values, then update the motors
               with the new PID value with each new output value.
             */

            /* Drive straight forward at 1/2 of full drive speed */
            double drive_speed = 0.5;

            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(drive_speed);
                    rightMotor.setPower(drive_speed);
                    telemetry.addData("Motor Output", df.format(drive_speed) + ", " +
                            df.format(drive_speed));
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(limit(drive_speed + output));
                    rightMotor.setPower(limit(drive_speed - output));
                    telemetry.addData("Motor Output", df.format(limit(drive_speed + output)) + ", " +
                            df.format(limit(drive_speed - output)));
                }
            } else {
                /* No sensor update has been received since the last time  */
                /* the loop() function was invoked.  Therefore, there's no */
                /* need to update the motors at this time.                 */
            }
            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
        }
    }
    public void drive_straight_lin (){
        int DEVICE_TIMEOUT_MS = 500;
        double drive_speed = 0.5;
        try {
            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftMotor.setPower(drive_speed);
                        rightMotor.setPower(drive_speed);
                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftMotor.setPower(drive_speed + output);
                        rightMotor.setPower(drive_speed - output);
                        telemetry.addData("PIDOutput", df.format(limit(drive_speed + output)) + ", " +
                                df.format(limit(drive_speed - output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                } else{
			        /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void turn_to_angle (){
        if ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if ( calibration_complete ) {
                navx_device.zeroYaw();
            } else {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        } else {
            /* Wait for new Yaw PID output values, then update the motors
               with the new PID value with each new output value.
             */
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
            /* No sensor update has been received since the last time  */
            /* the loop() function was invoked.  Therefore, there's no */
            /* need to update the motors at this time.                 */
            }
            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
        }
    }
    @Override
    public void stop() {
        navx_device.close();
    }// End turnDistance
}


