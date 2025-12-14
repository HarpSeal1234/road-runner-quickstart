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

package org.firstinspires.ftc.teamcode.TestFiles;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_DOWN;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="outtake", group="Linear OpMode")
public class outtakeonly extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx outtake1 = null;
    double targetOuttakeVelocity = 0.0;


    // pidf
    private double outtakeZeroPower = 0.0;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneTargetVelocity = 1300;
    private double motorOneMaxVelocity = 2800;
    private double F = 32767/motorOneMaxVelocity;
    private double kP = 1.25;
    private double kI = kP * 0.1;
    private double kD = kP * 0.01;
    private double position = 5.0;
    double blockerPositionR = R_BLOCKER_DOWN;
    double blockerPositionL = L_BLOCKER_DOWN;
    double pivotPosition = FAR_PIVOT_POSITION;

//    private Servo pivot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initHardware();


//        pivot = hardwareMap.get(Servo.class, "pivot");
//
//        pivot.setPosition(Range.clip(pivotPosition, 0.0, 1.0));


//        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(targetOuttakeVelocity);


        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        float step = 5;
        double outtakeStep = 7.0;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //drive
            double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double leftFrontPower = Range.clip((y + x + rx),-DRIVE_POWER,DRIVE_POWER);
            double leftBackPower = Range.clip((y - x + rx),-DRIVE_POWER,DRIVE_POWER);
            double rightFrontPower = Range.clip((y - x - rx),-DRIVE_POWER,DRIVE_POWER);
            double rightBackPower = Range.clip((y + x - rx),-DRIVE_POWER,DRIVE_POWER);


            //controls
            if(gamepad2.left_bumper) {
                outtake1.setPower(1.0);
            } else if(gamepad2.right_bumper) {
//                outtake1.setVelocity(CLOSE_OUTTAKE_VELOCITY);
            } else if (gamepad1.a) {
                outtake1.setVelocity(1200);
            } else if (gamepad1.x) {
                outtake1.setVelocity(1300);
            } else if (gamepad1.y) {
                outtake1.setVelocity(1400);
            }else if (gamepad1.b) {
                outtake1.setVelocity(1500);
            } else if (gamepad2.dpad_right) {
                outtake1.setVelocity(0.0);
            } else if (gamepad2.dpad_up){
                targetOuttakeVelocity = targetOuttakeVelocity + step;
                outtake1.setVelocity(Range.clip(targetOuttakeVelocity,0.0, FAR_OUTTAKE_VELOCITY));
            }else if (gamepad2.dpad_down){
                targetOuttakeVelocity = targetOuttakeVelocity - step;
                outtake1.setVelocity(Range.clip(targetOuttakeVelocity,0.0, FAR_OUTTAKE_VELOCITY));
            }
            motorTelemetry();
        }
    }
    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
    }

    private void initMotorOne(double kP, double kI, double kD, double F, double position) {
        outtake1  = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.REVERSE); // try reverse
        outtake1.setPower(outtakeZeroPower);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake1.setPositionPIDFCoefficients(position);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void motorTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Current Position", "Position: " + pivotPosition);
        telemetry.addData("Target Velocity", targetOuttakeVelocity);
        telemetry.addData("power", outtake1.getPower());
        telemetry.addData("Outtake Velocity", outtake1.getVelocity());
        telemetry.addData("F", F);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD","%.5f", kD);
//        telemetry.addData("rightFront", rightFrontPower);
//        telemetry.addData("leftFront", leftFrontPower);
//        telemetry.addData("rightBack", rightBackPower);
//        telemetry.addData("leftBack", leftBackPower);
        telemetry.update();
    }
}

