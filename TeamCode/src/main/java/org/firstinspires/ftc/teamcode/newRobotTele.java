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

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kD;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kI;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


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

@TeleOp(name="Tele baby robot uv", group="! Linear OpMode")
public class newRobotTele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;

    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DcMotorEx outtake1 = null;
        private DcMotorEx outtake2 = null;
    private DcMotorEx frontIntake = null;
    private DcMotor backIntake = null;
    double frontIntakePower = 0.0;
    double backIntakePower = 0.0;
    double firstIntake = 0.9;
    double secondIntake = 0.6;
    private Servo blocker;
    //    private Servo blockerL;
    double targetOuttakeVelocity = 0.0;

    // sensors
//    private NormalizedColorSensor r_frontColorSensor;
//    private NormalizedColorSensor l_frontColorSensor;
//    private NormalizedColorSensor r_backColorSensor;
//    private NormalizedColorSensor l_backColorSensor;

    // pidf
    private double outtakeZeroPower = 0.0;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneTargetVelocity = 1300;
    private double motorOneMaxVelocity = 2800;
    private double F = 32767/motorOneMaxVelocity;

    private double position = 5.0;
    double blockPosition = 0.6;
            ;
    double openPosition = 1;
    int r_frontHasBall = 0;
    int l_frontHasBall = 0;
    int r_backHasBall = 0;
    int l_backHasBall = 0;
    String pattern = "";

    int waitTime = 250;
    boolean intake1On = false;
    double intake1Vel = 0.0;
    private Servo pivot;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initHardware();

        float step = 0.001f;
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
            ElapsedTime intake1Timer = new ElapsedTime();

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);


            //controls
            if(gamepad2.left_bumper) {
                outtake1.setVelocity(FAR_OUTTAKE_VELOCITY);
//                outtake2.setVelocity(FAR_OUTTAKE_VELOCITY);
            } else if(gamepad2.right_bumper) {
                outtake1.setVelocity(CLOSE_OUTTAKE_VELOCITY);
//                outtake2.setVelocity(CLOSE_OUTTAKE_VELOCITY);
            } else if (gamepad2.dpad_right) {
                outtake1.setVelocity(0.0);
//                outtake2.setVelocity(0.0);
            }

            // INTAKE
            if (gamepad1.dpad_left){
                frontIntakePower = -0.7;
            }  else if (gamepad1.right_bumper) {
                frontIntakePower = 1.0;
//                intake1Timer.reset();
//                intake1On = true;
            } else if (gamepad1.left_bumper){
                frontIntakePower = 0.0;
                intake1On = false;
            } else if (gamepad1.a){
                backIntakePower = -1.0;
            } else if (gamepad1.b){
                backIntakePower = 0.0;
            }

            if (intake1On && (intake1Vel < 200) && (intake1Timer.seconds() > 0.3)){
                frontIntakePower = 0.0;
                intake1On = false;
                intake1Timer.reset();
            }

            intake1Vel = frontIntake.getVelocity();

            frontIntake.setPower(frontIntakePower);
            backIntake.setPower(backIntakePower);
            telemetry();

            telemetry.update();
        }

    }
    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
        initMotorTwo(kP, kI, kD, F, position);
        initDriveMotors();
        initIntake();
    }
    public void initDriveMotors(){
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initIntake(){
        frontIntake = hardwareMap.get(DcMotorEx.class,"intake1");
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backIntake = hardwareMap.get(DcMotor.class,"intake2");
        backIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initMotorOne(double kP, double kI, double kD, double F, double position) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setPower(outtakeZeroPower);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(targetOuttakeVelocity);
    }

    private void initMotorTwo(double kP, double kI, double kD, double F, double position) {
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.FORWARD);
        outtake2.setPower(outtakeZeroPower);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(targetOuttakeVelocity);
    }

    public void telemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Intake Power", "Intake Power: " + frontIntakePower);
        telemetry.addData("Target Velocity", targetOuttakeVelocity);
        telemetry.addData("Intake Vel", intake1Vel);
        telemetry.addData("Outtake 1 power", outtake1.getPower());
        telemetry.addData("Outtake 2 power", outtake2.getPower());
        telemetry.addData("Outtake 1 Velocity", outtake1.getVelocity());
        telemetry.addData("Outtake 2 Velocity", outtake2.getVelocity());
        telemetry.addData("F", F);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD","%.5f", kD);
    }
}

