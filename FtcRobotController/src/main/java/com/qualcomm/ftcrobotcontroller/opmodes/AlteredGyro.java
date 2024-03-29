package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Aaron on 10/17/2015.
 */
public class AlteredGyro extends OpMode {
    GyroSensor gyro;
    double gyro_default;
    double curr_gyro;
    double rotation = 0;
    double start = 0;
    double end = 0;

    public AlteredGyro(){

    }

    @Override
    public void init(){
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro_default = gyro.getRotation();
    }

    @Override
    public void loop(){
        if(end != 0){
            start = end;
        }else {
            start = System.currentTimeMillis();
        }
        curr_gyro = gyro.getRotation();
        telemetry.addData("Feedback", Double.toString(rotation));
        end = System.currentTimeMillis();
        if(Math.abs(curr_gyro - gyro_default) > 1) {
            rotation += (curr_gyro - gyro_default) * (end - start) / 1000;
        }
    }

    @Override
    public void stop(){

    }
}
