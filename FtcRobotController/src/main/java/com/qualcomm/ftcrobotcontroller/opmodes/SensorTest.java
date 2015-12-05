package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Cormac on 11/23/2015.
 */
public class SensorTest extends PushBotTelemetrySensors {
    public SensorTest() {

    }

    @Override
    public void init() {
        super.init();

        try {
            v_sensor_ods = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        } catch (Exception p_exeception) {
            try {
                v_sensor_ods = hardwareMap.opticalDistanceSensor.get
                        ("sensor_eopd"
                        );
            } catch (Exception p_exeception_eopd) {
                try {
                    v_sensor_ods = hardwareMap.opticalDistanceSensor.get
                            ("sensor_EOPD"
                            );
                } catch (Exception p_exeception_EOPD) {
                    m_warning_message("sensor_ods/eopd/EOPD");
                    DbgLog.msg
                            ("Can't map sensor_ods nor sensor_eopd, nor" +
                                            " sensor_EOPD ("
                                            + p_exeception_EOPD.getLocalizedMessage()
                                            + ").\n"
                            );
                    v_sensor_ods = null;
                }
            }

        }
    }

    @Override
    public void start() {
        super.start();
        reset_drive_encoders();
    }

    @Override
    public void loop() {

    }
}

