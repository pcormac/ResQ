/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.FieldPosition;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;

//import java.lang.annotation.Target;

public class blue extends PushBotTelemetry {

    DcMotor motorRight;
    DcMotor motorLeft;
    Servo lift;

    final static int ENCODER_CPR = 1;
    final static double GEAR_RATIO = 0.5;
    final static double WHEEL_DIAMETER = 20.41;
    final static double DISTANCE1 = 8; //two tiles
    final static double DISTANCE2 = 2; //half tile
    final static double DISTANCE3 = 8; //two tiles
    final static double DISTANCE4 = 8; //two tiles

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS1 = DISTANCE1 / CIRCUMFERENCE;
    final static double ROTATIONS2 = DISTANCE2 / CIRCUMFERENCE;
    final static double ROTATIONS3 = DISTANCE3 / CIRCUMFERENCE;
    final static double ROTATIONS4 = DISTANCE4 / CIRCUMFERENCE;

    final static double COUNTS1 = ENCODER_CPR * ROTATIONS1 * GEAR_RATIO;
    final static double COUNTS2 = ENCODER_CPR * ROTATIONS2 * GEAR_RATIO;
    final static double COUNTS3 = ENCODER_CPR * ROTATIONS3 * GEAR_RATIO;
    final static double COUNTS4 = ENCODER_CPR * ROTATIONS4 * GEAR_RATIO;


    int v_state = 1;
/*
    final static int ENCODER_CPR = 1440; //Encoder Counts per Revolution
    final static double GEAR_RATIO = 1; //Gear Ratio
    final static int WHEEL_DIAMETER = 3; //Diameter of the wheel in inches
    final static int DISTANCE = 75; //Distance in inches to drive

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
*/


    public blue() {

    }

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        lift = hardwareMap.servo.get("servo_2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);


    }

    @Override
    public void start() {


        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        try {
            Thread.sleep(7000);
            }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
/*
        switch (v_state) {
            case 1:
                telemetry.addData("Here 1", motorLeft.getCurrentPosition());

                motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

                //reset_drive_encoders();
                motorLeft.setTargetPosition(11459);
                motorRight.setTargetPosition(11459);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);


                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                //motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (Math.abs (motorLeft.getCurrentPosition ()) >= 11400) {
//                if (have_drive_encoders_reached(11459, 11459)) {
                    telemetry.addData("Here 2", motorLeft.getCurrentPosition());
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    v_state++;
                }
                break;
            case 2:

                motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

                //reset_drive_encoders();
                motorLeft.setTargetPosition(2000);
                motorRight.setTargetPosition(2000);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

                motorLeft.setPower(-0.5);
                motorRight.setPower(-0.5);

                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                //motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (have_drive_encoders_reached(2000, 2000)) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    v_state++;
                    break;
                }

            case 3:

                liftPositiion = 0;
                v_state++;
                break;

            case 4:
                motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorLeft.setTargetPosition(6110);
                motorRight.setTargetPosition(6110);

                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);


                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                // motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (have_drive_encoders_reached(6110, 6110))
                {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    v_state++;
                    break;
                }
        }

*/
    }





    @Override
    public void loop() {

        switch (v_state) {
            case 1:
                telemetry.addData("Here 1", motorLeft.getCurrentPosition());

//                motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//                motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

                //reset_drive_encoders();
                motorLeft.setTargetPosition(11459);
                motorRight.setTargetPosition(11459);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);


                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                //motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (Math.abs (motorLeft.getCurrentPosition ()) >= 11400) {
//                if (have_drive_encoders_reached(11459, 11459)) {
                    telemetry.addData("Here 2", motorLeft.getCurrentPosition());
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
 //                   try {
 //                       Thread.sleep(1000);
 //                   } catch (InterruptedException e) {
 //                       e.printStackTrace();
 //                   }
                    motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                    v_state++;
                }
                break;
            case 2:
                if (motorRight.getTargetPosition() == 0 && motorLeft.getTargetPosition() == 0)
                    v_state++;
                break;
            case 3:

//                motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//                motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

//                reset_drive_encoders();
                motorLeft.setTargetPosition(-2000);
                motorRight.setTargetPosition(-2000);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

                motorLeft.setPower(-0.5);
                motorRight.setPower(-0.5);

                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                //motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (Math.abs (motorLeft.getCurrentPosition ()) >= 1980) {
//                if (have_drive_encoders_reached(-2000, -2000)) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                    v_state++;
                }
                break;

            case 4:
                if (motorRight.getTargetPosition() == 0 && motorLeft.getTargetPosition() == 0)
                    v_state++;
                break;
//            case 5:

  //              liftPositiion = 0;
    //            v_state++;
      //          break;
            case 5:
                motorLeft.setTargetPosition(-1000);
                motorRight.setTargetPosition(1000);

                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);


                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                // motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (Math.abs (motorLeft.getCurrentPosition ()) >= 1000)
                {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                    v_state++;
                }
                break;
            case 6:
                if (motorRight.getTargetPosition() == 0 && motorLeft.getTargetPosition() == 0)
                    v_state++;
                break;
            case 7:

                lift.setPosition(0);
                v_state++;
                break;

            case 8:
                motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorLeft.setTargetPosition(-6110);
                motorRight.setTargetPosition(-6110);

                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);

                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);


                //motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                // motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
                if (have_drive_encoders_reached(6110, 6110))
                {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    v_state++;
                }
                break;
        }



        //telemetry.addData("Motor Target", );
        telemetry.addData("Left Position", motorLeft.getCurrentPosition());
        telemetry.addData("Right Position", motorRight.getCurrentPosition());
        telemetry.addData("Vstate", v_state);
    }
}