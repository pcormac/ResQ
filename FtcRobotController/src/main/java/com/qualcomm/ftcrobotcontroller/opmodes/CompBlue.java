package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;
import java.util.Timer;

import com.qualcomm.ftcrobotcontroller.opmodes.navXDriveStraightPIDLoopOp;

/**
 * Created by Cormac on 12/3/2015.
 */
public class CompBlue extends PushBotTelemetry
{
    public CompBlue () {}
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor arm;
    DcMotor wormy;
    //Servo leftServo;
    //Servo rightServo;
    //Servo armServo;
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
    private double TOTAL_RUN_TIME = 1;
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
        //leftServo = hardwareMap.servo.get("left_servo");
        //rightServo = hardwareMap.servo.get("right_servo");
        //armServo = hardwareMap.servo.get("arm_servo");

        //reverse right motor so forward is forward
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        //rightServo.setDirection(Servo.Direction.REVERSE);

        //leftServo.setPosition(.1);
        //rightServo.setPosition(.1);
        //armServo.setPosition(0);

        hardwareMap.logDevices();
        
//        update_telemetry(); // Update common telemetry

    }
    @Override public void start() {
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }

    @Override public void loop() {

        switch (v_state) {
            case 0:
                telemetry.addData("Case:", "0");
                try {
                    runtime.wait(500); //8 sec
                }
                catch (InterruptedException e){
                    e.printStackTrace();
                }
                // Transition to the next state when this method is called again.
                v_state++;
                break;
            case 1:
                telemetry.addData("Case:", "1");
                TOTAL_RUN_TIME = 5.0;
                runtime.reset();
                drive_straight_lin(TOTAL_RUN_TIME);
                update_telemetry();
                v_state++;
                break;
            case 2:
                telemetry.addData("Case:", "2");
                TARGET_ANGLE_DEGREES = 45.0;
                turn_to_angle(TARGET_ANGLE_DEGREES);
                update_telemetry();
                v_state++;
                break;
            case 3:
                telemetry.addData("Case:", "3");
                runtime.reset();
                TOTAL_RUN_TIME = 3;
                drive_straight_lin(TOTAL_RUN_TIME);
                update_telemetry();
                v_state++;
                break;
            case 4:
                telemetry.addData("Case:", "4");
                TARGET_ANGLE_DEGREES = 45.0;
                turn_to_angle(TARGET_ANGLE_DEGREES);
                update_telemetry();
                v_state++;
                break;
            case 5:
                telemetry.addData("Case:", "5");
                runtime.reset();
                TOTAL_RUN_TIME = 2; //11+1.5
                drive_straight_lin(TOTAL_RUN_TIME);
                update_telemetry();
                v_state++;
                break;
            case 6:
                telemetry.addData("Case:", "6");
                //armServo.setPosition(.7);
                try {
                    Thread.sleep(500); //1/2 sec
                }
                catch (InterruptedException e){
                    e.printStackTrace();
                }
                //armServo.setPosition(0);
                TARGET_ANGLE_DEGREES = 180;
                turn_to_angle(TARGET_ANGLE_DEGREES);
                update_telemetry();
                v_state++;
                break;
            case 7:
                telemetry.addData("Case:", "7");
                runtime.reset();
                TOTAL_RUN_TIME = 4;
                drive_straight_backwards_lin(TOTAL_RUN_TIME);
                update_telemetry();
                v_state++;
                break;
            case 8:
                telemetry.addData("Case:", "8");
                TARGET_ANGLE_DEGREES = -60.0;
                turn_to_angle(TARGET_ANGLE_DEGREES);
                update_telemetry();
                v_state++;
                break;
            case 9:
                telemetry.addData("Case:", "9");
                runtime.reset();
                TOTAL_RUN_TIME = 4;
                drive_straight_lin(TOTAL_RUN_TIME);
                update_telemetry();
                v_state++;
                break;
            case 10:
                telemetry.addData("Case:", "10");
                TARGET_ANGLE_DEGREES = 10.0;
                turn_to_angle(TARGET_ANGLE_DEGREES);
                update_telemetry();
                v_state++;
                break;
            case 11:
                telemetry.addData("Case:", "11");
                runtime.reset();
                TOTAL_RUN_TIME = 5;
                drive_straight_lin(TOTAL_RUN_TIME);
                update_telemetry();
                v_state++;
                break;
            case 12:
                telemetry.addData("Case:", "Done");
                update_telemetry();
                break;
        }
        update_telemetry(); // Update common telemetry
    }

    public void update_telemetry () {
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
                 );
        telemetry.addData
                ("02"
                        , "Right Drive: "
                                + a_right_drive_power()
                );
        telemetry.addData("Case:", v_state);
        telemetry.addData("Time:", Math.round(getRuntime()));
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
    
    public void drive_straight_lin(double TOTAL_RUN_TIME){
        int DEVICE_TIMEOUT_MS = 500;
        double drive_speed = 0.5;
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
        try {
            while ((runtime.time() < TOTAL_RUN_TIME) &&
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

    public void drive_straight_backwards_lin (double TOTAL_RUN_TIME) {
        int DEVICE_TIMEOUT_MS = 500;
        double drive_speed = -0.5;
        try {
            while ((runtime.time() < TOTAL_RUN_TIME) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftMotor.setPower(drive_speed);
                        rightMotor.setPower(drive_speed);
                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftMotor.setPower(drive_speed - output);
                        rightMotor.setPower(drive_speed + output);
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

    public void turn_to_angle (double TARGET_ANGLE_DEGREES) {
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
            /* need to update the motors at this time.       3          */
                telemetry.addData("Gyro", "No new updates");
            }
            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
        }
    }
    @Override
    public void stop() {
        navx_device.close();
    }// End turnDistance
}


