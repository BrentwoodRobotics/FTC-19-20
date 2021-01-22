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


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import com.qualcomm.robotcore.hardware.Servo;

/*
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;*/

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name="BLUE_Block_Bridge", group="A")
public class BLUE_Block_Bridge extends LinearOpMode {


    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // Declare OpMode motors
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor rightFront = null;
    private Servo blockGrab = null;
    private Servo armDrag1 = null;
    private DcMotor linearActuator = null;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        //Expansion Hub 1 (driving controls)
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        blockGrab = hardwareMap.get(Servo.class, "blockGrab");
        armDrag1 = hardwareMap.get(Servo.class,"armDrag1" );
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");


        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Get heading while on lander
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        telemetry.addData("First Angle: ", startHeading);
        telemetry.addData("Status: ", "Init");
        telemetry.update();

        //arm goes up a little
        linearActuator.setPower(-0.6);
        sleep(100);
        linearActuator.setPower(0);

        blockGrab.setPosition(1);
        sleep(400);

        //drives to block
        leftFront.setPower(-.5);
        rightFront.setPower(-.5);
        leftRear.setPower(-.5);
        rightRear.setPower(-.5);
        sleep(1000);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(1000);

        //claw grabs block
        blockGrab.setPosition(.2);
        sleep(1000);

        //arm goes up a little
        linearActuator.setPower(0.6);
        sleep(100);
        linearActuator.setPower(0);

        //backs up a little
        leftFront.setPower(.5);
        rightFront.setPower(.5);
        leftRear.setPower(.5);
        rightRear.setPower(.5);
        sleep(150);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;
        telemetry.addData("before turn Angle: ", currentHeading);
        telemetry.addData("Status: ", "Init");
        telemetry.update();

        // Aims towards new heading
        while (currentHeading < 78)
        {
            leftFront.setPower(.2);
            rightFront.setPower(-.2);
            leftRear.setPower(.2);
            rightRear.setPower(-.2);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            telemetry.addData("angle turn ", currentHeading);
            //telemetry.addData("Status: ", "Init");
            telemetry.update();
        }


        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(1000);


        //goes straight, under bridge
        leftFront.setPower(-.4);
        rightFront.setPower(-.4);
        leftRear.setPower(-.4);
        rightRear.setPower(-.4);
        sleep(1500);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(750);

        //drops block
        blockGrab.setPosition(1);
        sleep(2000);

        linearActuator.setPower(-0.6);
        sleep(100);
        linearActuator.setPower(0);

        //goes backwards, under bridge
        leftFront.setPower(.4);
        rightFront.setPower(.4);
        leftRear.setPower(.4);
        rightRear.setPower(.4);
        sleep(1880);

        linearActuator.setPower(0.6);
        sleep(100);
        linearActuator.setPower(0);

        // Aims towards new heading
        while (currentHeading > 13)
        {
            leftFront.setPower(-.2);
            rightFront.setPower(.2);
            leftRear.setPower(-.2);
            rightRear.setPower(.2);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            telemetry.addData("angle turn ", currentHeading);
            //telemetry.addData("Status: ", "Init");
            telemetry.update();
        }

        //goes straight, to block
        leftFront.setPower(-.4);
        rightFront.setPower(-.4);
        leftRear.setPower(-.4);
        rightRear.setPower(-.4);
        sleep(275);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(750);

        //grabs block
        blockGrab.setPosition(.2);
        sleep(2000);

        leftFront.setPower(.4);
        rightFront.setPower(.4);
        leftRear.setPower(.4);
        rightRear.setPower(.4);
        sleep(250);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(750);

        // Aims towards new heading
        while (currentHeading < 78)
        {
            leftFront.setPower(.2);
            rightFront.setPower(-.2);
            leftRear.setPower(.2);
            rightRear.setPower(-.2);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            telemetry.addData("angle turn ", currentHeading);
            //telemetry.addData("Status: ", "Init");
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(1000);


        //goes straight, under bridge
        leftFront.setPower(-.4);
        rightFront.setPower(-.4);
        leftRear.setPower(-.4);
        rightRear.setPower(-.4);
        sleep(1800);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(750);

        //drops block
        blockGrab.setPosition(1);

        linearActuator.setPower(-0.6);
        sleep(100);
        linearActuator.setPower(0);

        leftFront.setPower(.4);
        rightFront.setPower(.4);
        leftRear.setPower(.4);
        rightRear.setPower(.4);
        sleep(400);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(200);
        blockGrab.setPosition(.2);

        leftFront.setPower(-.4);
        rightFront.setPower(.4);
        leftRear.setPower(.4);
        rightRear.setPower(-.4);
        sleep(400);

    }}






