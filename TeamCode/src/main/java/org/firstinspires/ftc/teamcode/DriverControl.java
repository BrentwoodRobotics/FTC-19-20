/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *h
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControl", group="Linear Opmode")
//@Disabled
public class DriverControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor rightFront = null;
    private Servo blockGrab = null;
    private Servo armDrag1 = null;
    private Servo armDrag2 = null;
    private DcMotor linearLift = null;
    private DcMotor linearActuator = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        blockGrab = hardwareMap.get(Servo.class, "blockGrab");
        armDrag1 = hardwareMap.get(Servo.class,"armDrag1" );
        armDrag2 = hardwareMap.get(Servo.class,"armDrag2" );
        linearLift = hardwareMap.get(DcMotor.class, "linearLift");
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /* leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE); */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double speedLimiter = 1.0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            if(gamepad1.right_bumper)
            {
                speedLimiter = 0.3;
            }

            if(gamepad1.left_bumper)
            {
                speedLimiter = 1.0;
            }

            double r = Math.hypot(gamepad1.left_stick_y,-gamepad1.left_stick_x);

            double robotAngle = Math.atan2(gamepad1.left_stick_y,-gamepad1.left_stick_x)- Math.PI / 4;

            double v1 = r * Math.cos(robotAngle);

            double v2 = r * Math.sin(robotAngle);

            double v3 = r * Math.sin(robotAngle);

            double v4 = r * Math.cos(robotAngle);

            /*
            v1  = Range.clip(v1, -1.0, 1.0) ;
            v2  = Range.clip(v2, -1.0, -1.0) ;
            v3  = Range.clip(v3, 1.0, 1.0) ;
            v4  = Range.clip(v4, 1.0, -1.0) ;
            */

            leftFront.setPower((v1 - gamepad1.right_stick_x)*speedLimiter);
            leftRear.setPower((v2 - gamepad1.right_stick_x)*speedLimiter);
            rightFront.setPower((v3 + gamepad1.right_stick_x)*speedLimiter);
            rightRear.setPower((v4 + gamepad1.right_stick_x)*speedLimiter);


            if(gamepad2.y)
            {
                armDrag1.setPosition(0);
            }
            else
            {
                armDrag1.setPosition(1);
            }

            if(gamepad2.b)
            {
                blockGrab.setPosition(1);
            }


            if(gamepad2.a)
            {
                blockGrab.setPosition(.2);
            }

            if(gamepad2.right_bumper)
            {
            linearActuator.setPower(0.5);
            }

            if(gamepad2.left_bumper) {
                linearActuator.setPower(-0.5);
            }
            else
            {
                linearActuator.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front", leftFront.getPower());
            telemetry.addData("Right Front", rightFront.getPower());
            telemetry.addData("Left Rear", leftRear.getPower());
            telemetry.addData("Right Rear", rightRear.getPower());
            telemetry.addData("blockGrab", blockGrab.getPosition());
            telemetry.addData("armDrag1", armDrag1.getPosition());
            telemetry.addData("armDrag2", armDrag2.getPosition());
            telemetry.update();
        }
    }
}
