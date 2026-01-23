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

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.ACTUAL_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.DRIVE_POWER;
//import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.F;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.INTAKE_ZERO_POWER;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_UP;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_UP;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kD;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kI;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kP;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


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

@TeleOp(name="AA_decode1024", group="! Linear OpMode")
public class Tele extends LinearOpMode {
    enum LAUNCH_STATUS {
        NOT_LAUNCHED,
        LAUNCHED
    }

    enum INTAKE_STATUS {
        INTAKE_STOPPED,
        INTAKE_STARTED,
        INTAKE_NEED_TO_BE_MONITORED,
        INTAKE_MONITORING,
        INTAKE_JAMMED
    }
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;

    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DcMotorEx outtake1 = null;
    private DcMotorEx outtake2 = null;
    private DcMotorEx intake1 = null;
    double intakePower = 0.0;
    private Servo blockerR;
    private Servo blockerL;
    double targetOuttakeVelocity = 0.0;

    // sensors
    private NormalizedColorSensor r_frontColorSensor;
    private NormalizedColorSensor l_frontColorSensor;
    private NormalizedColorSensor r_backColorSensor;
    private NormalizedColorSensor l_backColorSensor;

    // pidf
    private double outtakeZeroPower = 0.0;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneTargetVelocity = 1300;
    private double motorOneMaxVelocity = 2800;
    private double F = 32767/motorOneMaxVelocity;

    // blinkin
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern;


    private double position = 5.0;
    double blockerPositionR = R_BLOCKER_DOWN;
    double blockerPositionL = L_BLOCKER_DOWN;
//    double pivotPosition = FAR_PIVOT_POSITION;

    NormalizedRGBA r_frontColor;
    NormalizedRGBA r_backColor;
    NormalizedRGBA l_frontColor;
    NormalizedRGBA l_backColor;
    /*
    boolean r_frontHasBall = false;
    boolean r_frontGreen = false;

    boolean l_frontHasBall = false;
    boolean l_frontGreen = false;

    boolean r_backHasBall = false;
    boolean r_backGreen = false;

    boolean l_backHasBall = false;
    boolean l_backGreen = false;*/
    int r_frontHasBall = 0;
    int l_frontHasBall = 0;
    int r_backHasBall = 0;
    int l_backHasBall = 0;
    LAUNCH_STATUS leftLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;
    LAUNCH_STATUS rightLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;
//    int pattern = 0;


    double r_frontDistance = 0.0;
    double l_frontDistance = 0.0;
    double r_backDistance = 0.0;
    double l_backDistance = 0.0;
    int waitTime = 250;
    double intake1Vel = 0;
    INTAKE_STATUS intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
//    private Servo pivot;
    private double targetv = 0;
    boolean autoUpdate = false;

    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initAprilTag();
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();


        initHardware();


        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW;
        blinkin.setPattern(blinkinPattern);

//        pivot = hardwareMap.get(Servo.class, "pivot");
//        pivot.setPosition(Range.clip(pivotPosition, 0.0, 1.0));



        // Wait for the game to start (driver presses START)
        float step = 0.001f;
        double outtakeStep = 7.0;
        boolean rightBlockerEngaged = false;
        boolean leftBlockerEngaged = false;
        ElapsedTime rightBlockerTimer = new ElapsedTime();
        ElapsedTime leftBlockerTimer = new ElapsedTime();
        ElapsedTime patternTimer = new ElapsedTime();
        ElapsedTime velocityTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();
        ElapsedTime ballIntake = new ElapsedTime();
        double lastTargetV = 0;



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //drive
            double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double leftFrontPower = Range.clip((y + x + rx), -DRIVE_POWER, DRIVE_POWER);
            double leftBackPower = Range.clip((y - x + rx), -DRIVE_POWER, DRIVE_POWER);
            double rightFrontPower = Range.clip((y - x - rx), -DRIVE_POWER, DRIVE_POWER);
            double rightBackPower = Range.clip((y + x - rx), -DRIVE_POWER, DRIVE_POWER);
            targetv = Range.clip((((1450.0 - 1100) / (130 - 42)) * (getRobotToGoalDistance() - 42) + 1100), 900, FAR_OUTTAKE_VELOCITY);
            intake1Vel = intake1.getVelocity();

//            telemetryAprilTag();
//            telemetry.update();

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // color sensor
//            r_frontColor = r_frontColorSensor.getNormalizedColors();
//            r_frontColor.toColor();
//            r_backColor = r_backColorSensor.getNormalizedColors();
//            r_backColor.toColor();
//            l_frontColor = l_frontColorSensor.getNormalizedColors();
//            l_frontColor.toColor();
//            l_backColor = l_backColorSensor.getNormalizedColors();
//            l_backColor.toColor();
//            if (r_frontColorSensor instanceof DistanceSensor) {
//                r_frontDistance = ((DistanceSensor) r_frontColorSensor).getDistance(DistanceUnit.CM);
//                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) r_frontColorSensor).getDistance(DistanceUnit.CM));
//            }

            //controls
            // OUTTAKE
            if (gamepad2.left_bumper) {
                targetOuttakeVelocity = FAR_OUTTAKE_VELOCITY;
                autoUpdate = false;
            } else if (gamepad2.right_bumper) {
                targetOuttakeVelocity = CLOSE_OUTTAKE_VELOCITY;
                autoUpdate = false;
            } else if (gamepad2.y) {
                targetOuttakeVelocity = -200;
            } else if (gamepad2.dpad_right) {
                targetOuttakeVelocity = 0;
                autoUpdate = false;
            } else if (gamepad2.dpad_up) {
                autoUpdate = true;
                velocityTimer.reset();
                patternTimer.reset();
            }

/*
            //PIVOT
            if (gamepad1.x) {
                pivotPosition = ACTUAL_PIVOT_POSITION;
                pivotPosition = Range.clip(pivotPosition, 0.0, 1.0);
                pivot.setPosition(pivotPosition);
            } else if (gamepad1.y) {
                pivotPosition = FAR_PIVOT_POSITION;
                pivotPosition = Range.clip(pivotPosition, 0.0, 1.0);
                pivot.setPosition(pivotPosition);
            }

 */

            // AVOCADOS
            if (gamepad2.b) {
                blockerPositionR = R_BLOCKER_UP;
                rightLaunchStatus = LAUNCH_STATUS.LAUNCHED;
                blockerR.setPosition(Range.clip(blockerPositionR, 0.0, R_BLOCKER_UP));
                if (!rightBlockerEngaged) {
                    rightBlockerEngaged = true;
                    rightBlockerTimer.reset();
                }
            } else if (gamepad2.x) {
                blockerPositionL = L_BLOCKER_UP;
                blockerL.setPosition(Range.clip(blockerPositionL, 0.0, L_BLOCKER_DOWN));
                leftLaunchStatus = LAUNCH_STATUS.LAUNCHED;
                if (!leftBlockerEngaged) {
                    leftBlockerEngaged = true;
                    leftBlockerTimer.reset();
                }
            } else if (gamepad2.a) {
                blockerPositionL = L_BLOCKER_UP;
                blockerPositionR = R_BLOCKER_UP;
                leftLaunchStatus = LAUNCH_STATUS.LAUNCHED;
                rightLaunchStatus = LAUNCH_STATUS.LAUNCHED;

                blockerL.setPosition(Range.clip(blockerPositionL, 0.0, L_BLOCKER_DOWN));
                blockerR.setPosition(Range.clip(blockerPositionR, 0.0, R_BLOCKER_UP));
                if (!leftBlockerEngaged) {
                    leftBlockerEngaged = true;
                    leftBlockerTimer.reset();
                }
                if (!rightBlockerEngaged) {
                    rightBlockerEngaged = true;
                    rightBlockerTimer.reset();
                }
            }


            // INTAKE
            if (gamepad1.right_bumper) {
                intakePower = INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
            } else if (gamepad1.left_bumper) {
                intakePower = INTAKE_ZERO_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }
            if (gamepad1.dpad_left) {
                intakePower = -INTAKE_POWER;
            }
            intake1Vel = intake1.getVelocity();

            if (intakeStatus == INTAKE_STATUS.INTAKE_STARTED && intake1Vel > 800) {
                intakeStatus = INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED;
            }
            if (intakeStatus == INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED && intake1Vel < 200) {
                intakeStatus = INTAKE_STATUS.INTAKE_MONITORING;
                intakeTimer.reset();
            }
            if (intakeStatus == INTAKE_STATUS.INTAKE_MONITORING && intake1Vel < 200 && intakeTimer.milliseconds() > 100) {
                intakeStatus = INTAKE_STATUS.INTAKE_JAMMED;
            }
            if (intakeStatus == INTAKE_STATUS.INTAKE_JAMMED) {
                intakePower = 0;
                intakeTimer.reset();
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }

            // BLOCKER TIMERS
            if (leftBlockerEngaged) {
                if (leftBlockerTimer.milliseconds() > waitTime) {
                    blockerPositionL = L_BLOCKER_DOWN;
                    blockerL.setPosition(Range.clip(blockerPositionL, 0.0, L_BLOCKER_DOWN));
                    leftBlockerEngaged = false;
                }
            }
            if (rightBlockerEngaged) {
                if (rightBlockerTimer.milliseconds() > waitTime) {
                    blockerPositionR = R_BLOCKER_DOWN;
                    blockerR.setPosition(Range.clip(blockerPositionR, 0.0, R_BLOCKER_UP));
                    rightBlockerEngaged = false;
                }
            }
            /*
            if (velocityTimer.seconds() > 2) {
                targetv = Range.clip((((1450.0-1100)/(130-42))*(getRobotToGoalDistance()-42)+1100),900,FAR_OUTTAKE_VELOCITY);
                velocityTimer.reset();
            }

             */

            if (autoUpdate) {
                if (getGoal()) {
                    targetv = Range.clip((((1450.0 - 1100) / (130 - 42)) * (getRobotToGoalDistance() - 42) + 1100), 900, FAR_OUTTAKE_VELOCITY);
                    lastTargetV = targetv;
                    velocityTimer.reset();
                } else if (!getGoal() && (velocityTimer.seconds() > 1)) {
                    targetv = lastTargetV;
                } else {
                    targetv = Range.clip((((1450.0 - 1100) / (130 - 42)) * (getRobotToGoalDistance() - 42) + 1100), 900, FAR_OUTTAKE_VELOCITY);
                }
//                if (velocityTimer.seconds() > 1){
//                    targetv = Range.clip((((1450.0-1100)/(130-42))*(getRobotToGoalDistance()-42)+1100),900,FAR_OUTTAKE_VELOCITY);
//                    velocityTimer.reset();
//                }
                targetOuttakeVelocity = targetv;
            }
/*
            if (velocityTimer.seconds() > 2){
                if (autoUpdate) {
                    targetv = Range.clip((((1450.0-1100)/(130-42))*(getRobotToGoalDistance()-42)+1100),900,FAR_OUTTAKE_VELOCITY);
                    targetOuttakeVelocity = targetv;
                }
            }*/

            outtake1.setVelocity(Range.clip(targetOuttakeVelocity, -400, FAR_OUTTAKE_VELOCITY));
            outtake2.setVelocity(Range.clip(targetOuttakeVelocity, -400, FAR_OUTTAKE_VELOCITY));
            getRobotToGoalDistance();
            intake1.setPower(intakePower);
            telemetry();

//            checkColor(r_frontDistance,r_frontColor.red,r_frontColor.green,r_frontHasBall,r_frontGreen);
//            checkColor(r_backDistance,r_backColor.red,r_backColor.green,r_backHasBall,r_backGreen);
//            checkColor(l_frontDistance,l_frontColor.red,l_frontColor.green,l_frontHasBall,l_frontGreen);
//            checkColor(l_backDistance,l_backColor.red,l_backColor.green,l_backHasBall,l_backGreen);
//            /*
            if(rightLaunchStatus == LAUNCH_STATUS.NOT_LAUNCHED) {
                if (r_frontHasBall == 0) {
                    r_frontDistance = ((DistanceSensor) r_frontColorSensor).getDistance(DistanceUnit.CM);
                    r_frontHasBall = checkBallINT(r_frontDistance);
                    if(r_frontHasBall == 1){
                        r_backDistance = ((DistanceSensor) r_backColorSensor).getDistance(DistanceUnit.CM);
                        r_backHasBall = checkBallINT(r_backDistance);
                    }
                }
                if (r_backHasBall == 0) {
                    r_backDistance = ((DistanceSensor) r_backColorSensor).getDistance(DistanceUnit.CM);
                    r_backHasBall = checkBallINT(r_backDistance);
                    if (r_backHasBall == 1) {
                        r_frontDistance = ((DistanceSensor) r_frontColorSensor).getDistance(DistanceUnit.CM);
                        r_frontHasBall = checkBallINT(r_frontDistance);
                    }
                }
            }else{
                if (r_backHasBall == 1) {
                    r_backDistance = ((DistanceSensor) r_backColorSensor).getDistance(DistanceUnit.CM);
                    r_backHasBall = checkBallINT(r_backDistance);
                    if (r_backHasBall == 0) {
                        rightLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;

                        r_frontDistance = ((DistanceSensor) r_frontColorSensor).getDistance(DistanceUnit.CM);
                        r_frontHasBall = checkBallINT(r_frontDistance);
                    }
                }else{
                    r_backDistance = ((DistanceSensor) r_backColorSensor).getDistance(DistanceUnit.CM);
                    r_backHasBall = checkBallINT(r_backDistance);
                    if (r_backHasBall == 1) {
                        rightLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;

                        r_frontDistance = ((DistanceSensor) r_frontColorSensor).getDistance(DistanceUnit.CM);
                        r_frontHasBall = checkBallINT(r_frontDistance);
                    }
                }
            }
            if(leftLaunchStatus == LAUNCH_STATUS.NOT_LAUNCHED) {
                if (l_frontHasBall == 0) {
                    l_frontDistance = ((DistanceSensor) l_frontColorSensor).getDistance(DistanceUnit.CM);
                    l_frontHasBall = checkBallINT(l_frontDistance);
                    if (l_frontHasBall == 1) {
                        l_backDistance = ((DistanceSensor) l_backColorSensor).getDistance(DistanceUnit.CM);
                        l_backHasBall = checkBallINT(l_backDistance);
                    }
                }
                if (l_backHasBall == 0) {
                    l_backDistance = ((DistanceSensor) l_backColorSensor).getDistance(DistanceUnit.CM);
                    l_backHasBall = checkBallINT(l_backDistance);
                    if(l_backHasBall == 1) {
//                        l_frontHasBall = 0;
                        l_frontDistance = ((DistanceSensor) l_frontColorSensor).getDistance(DistanceUnit.CM);
                        l_frontHasBall = checkBallINT(l_frontDistance);
                    }
                }
            } else{
                leftLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;
                if (l_backHasBall == 1) {
                    l_backDistance = ((DistanceSensor) l_backColorSensor).getDistance(DistanceUnit.CM);
                    l_backHasBall = checkBallINT(l_backDistance);
                    if (l_backHasBall == 0) {
                        rightLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;

                        l_frontDistance = ((DistanceSensor) l_frontColorSensor).getDistance(DistanceUnit.CM);
                        l_frontHasBall = checkBallINT(l_frontDistance);
                    }
                }else{
                    l_backDistance = ((DistanceSensor) l_backColorSensor).getDistance(DistanceUnit.CM);
                    l_backHasBall = checkBallINT(l_backDistance);
                    if (l_backHasBall == 1) {
                        rightLaunchStatus = LAUNCH_STATUS.NOT_LAUNCHED;

                        l_frontDistance = ((DistanceSensor) l_frontColorSensor).getDistance(DistanceUnit.CM);
                        l_frontHasBall = checkBallINT(l_frontDistance);
                    }
                }
            }

//            blinkinColors(r_frontHasBall,r_backHasBall,l_frontHasBall,l_backHasBall,r_frontGreen,r_backGreen,l_frontGreen,l_backGreen);
//            telemetry.addData("r front", r_frontHasBall);
//            telemetry.addData("r back", r_backHasBall);
//            telemetry.addData("l front", l_frontHasBall);
//            telemetry.addData("l back", l_backHasBall);
//            telemetry.addData("distance", r_backDistance);
                blinkinINT(r_frontHasBall, r_backHasBall, l_frontHasBall, l_backHasBall);
                blinkinINT(r_frontHasBall, r_backHasBall, l_frontHasBall, l_backHasBall);
                blinkinINT(r_frontHasBall, r_backHasBall, l_frontHasBall, l_backHasBall);
                blinkinINT(r_frontHasBall, r_backHasBall, l_frontHasBall, l_backHasBall);

//            colorTelemetry(l_frontColor.red,l_frontColor.blue,l_frontColor.green,l_frontDistance);
//            individualTest(l_frontDistance, l_frontColor.red,l_frontColor.green);

            telemetry.update();
            blinkin.setPattern(blinkinPattern);
        }
        visionPortal.close();

    }
    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
        initMotorTwo(kP, kI, kD, F, position);
        initDriveMotors();
        initBlockers();
        initIntake();
        initColorSensors();
    }
    public void initIntake(){
        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

    public void initBlockers(){
        blockerR = hardwareMap.get(Servo.class, "blockerR");
        blockerL = hardwareMap.get(Servo.class, "blockerL");
        blockerR.setPosition(Range.clip(blockerPositionR, 0.0, 1.0));
        blockerL.setPosition(Range.clip(blockerPositionL, 0.0, 1.0));
    }

    private void initMotorOne(double kP, double kI, double kD, double F, double position) {
        outtake1  = hardwareMap.get(DcMotorEx.class, "outtake1");
//        outtake1.setDirection(DcMotor.Direction.FORWARD); /// NORMAL FORWARD -- 12.1 changed
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setPower(outtakeZeroPower);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake1.setPositionPIDFCoefficients(position);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(targetOuttakeVelocity);
    }
    private void initMotorTwo(double kP, double kI, double kD, double F, double position) {
        outtake2  = hardwareMap.get(DcMotorEx.class, "outtake2");
//        outtake1.setDirection(DcMotor.Direction.FORWARD); /// NORMAL FORWARD -- 12.1 changed
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setPower(outtakeZeroPower);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setPositionPIDFCoefficients(position);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(targetOuttakeVelocity);
    }
    private void initColorSensors() {
        // color sensor
        r_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_frontColorSensor");
        r_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_backColorSensor");
        l_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_frontColorSensor");
        l_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_backColorSensor");
    }

    private void blinkinColors(boolean r_frontHasBall, boolean r_backHasBall, boolean l_frontHasBall, boolean l_backHasBall,
                               boolean r_frontGreen, boolean r_backGreen, boolean l_frontGreen, boolean l_backGreen){
        if (!r_frontHasBall && !r_backHasBall && !l_frontHasBall && !l_backHasBall){
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else if((r_frontHasBall && r_backHasBall && l_backHasBall) || (l_frontHasBall && l_backHasBall && r_backHasBall)) { // 3 balls
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if((r_backHasBall && !r_frontHasBall && !l_backHasBall && !l_frontHasBall) || // rb
                (!r_backHasBall && !r_frontHasBall && l_backHasBall && !l_frontHasBall) || // lb
                (!r_backHasBall && r_frontHasBall && !l_backHasBall && !l_frontHasBall) || // rf
                (!r_backHasBall && !r_frontHasBall && !l_backHasBall && l_frontHasBall)){ // lf
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else if((r_backHasBall && r_frontHasBall && !l_backHasBall && !l_frontHasBall) ||//rf rb
                (!r_backHasBall && !r_frontHasBall && l_backHasBall && l_frontHasBall) || // lb lf
                (!r_backHasBall && r_frontHasBall && !l_backHasBall && l_frontHasBall) || // rf lf
                (r_backHasBall && !r_frontHasBall && l_backHasBall && !l_frontHasBall) || // rb lb
                (!r_backHasBall && r_frontHasBall && l_backHasBall && !l_frontHasBall) || // rf lb
                (r_backHasBall && !r_frontHasBall && !l_backHasBall && l_frontHasBall) ){ // rb lf
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
        }
    }

    public void blinkinINT(int r_frontHasBall, int r_backHasBall, int l_frontHasBall, int l_backHasBall){
        if ((r_frontHasBall + r_backHasBall + l_frontHasBall + l_backHasBall) == 0){
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        } else if ((r_frontHasBall + r_backHasBall + l_frontHasBall + l_backHasBall) == 1) {
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        } else if ((r_frontHasBall + r_backHasBall + l_frontHasBall + l_backHasBall) == 2) {
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else if ((r_frontHasBall + r_backHasBall + l_frontHasBall + l_backHasBall) == 3) {
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if ((r_frontHasBall + r_backHasBall + l_frontHasBall + l_backHasBall) == 4){
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }

    }

    /*private void checkColor(double distance, double red, double green, boolean hasBall, boolean isGreen){
        if (distance < 4.0) {
            hasBall = true;
            if ((green / red) > 2) {
                isGreen = true;
            } else if ((green / red) < 2){
                isGreen = false;
            }
        } else {
            hasBall = false;
        }
    }*/

    private boolean checkBall(double distance, boolean hasBall){
        if (distance < 4.0) {
            hasBall = true;
            return hasBall;
        } else {
            hasBall = false;
            return hasBall;
        }

    }

    private int checkBallINT(double distance){
        if (distance < 2.0) {
            return 1;
        } else {
            return 0;
        }

    }
    private void checkColor(double distance, double red, double green, boolean hasBall, boolean isGreen){
        if (distance < 4.0) {
            hasBall = true;
            if ((green / red) > 2) {
                isGreen = true;
            } else if ((green / red) < 2){
                isGreen = false;
            }
        } else {
            hasBall = false;
        }
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private double getRobotToGoalDistance(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        double distance = -1.0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24){
                    distance = detection.ftcPose.y;
                    break;
                }
            }
        }
        return distance;
    } // end method

    private boolean getGoal(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
//        double id = -1.0;
        boolean goal = false;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24){
                    goal = true;
                    break;
                }
            }
        }
        return goal;
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void colorTelemetry(double red, double blue, double green, double distance){
        telemetry.addData("red", red);
        telemetry.addData("blue", blue);
        telemetry.addData("green", green);
        telemetry.addData("distance", distance);
    }

    public void individualTest(double distance, double red, double green){
        if (distance < 4.0) {
            if ((green / red) > 2) {
                telemetry.addData("BALLCOLOR","green"); //r_frontColor.green / r_frontColor.red
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if ((green / red) < 2){
                telemetry.addData("BALLCOLOR","purple"); //r_frontColor.green / r_frontColor.red
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            }
        } else {
            telemetry.addData("BALLCOLOR","OTHER"); //r_frontColor.green / r_frontColor.red
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }
    }

    public void telemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("state", intakeStatus);
        telemetry.addData("left color state", leftLaunchStatus);
        telemetry.addData("right color state", rightLaunchStatus);
        telemetry.addData("Intake Power", "Intake Power: " + intakePower);
        telemetry.addData("blockerPosLEFT", "Position: " + blockerPositionL);
        telemetry.addData("blockerPosRIGHT", "Position: " + blockerPositionR);
//        telemetry.addData("Current Position", "Position: " + pivotPosition);
        telemetry.addData("Target Velocity", targetv);
        telemetry.addData("INTAKE Velocity", intake1.getVelocity());
        telemetry.addData("Outtake 1 power", outtake1.getPower());
        telemetry.addData("Outtake 2 power", outtake2.getPower());
        telemetry.addData("Outtake 1 Velocity", outtake1.getVelocity());
        telemetry.addData("Outtake 2 Velocity", outtake2.getVelocity());
//        telemetry.addData("F", F);
//        telemetry.addData("kP", kP);
//        telemetry.addData("kI", kI);
//        telemetry.addData("kD","%.5f", kD);
        telemetryAprilTag();
        telemetry.addData("rightFront", r_frontHasBall);
        telemetry.addData("leftFront", l_frontHasBall);
        telemetry.addData("rightBack", r_backHasBall);
        telemetry.addData("leftBack", l_backHasBall);
        telemetry.addData("rightFront distance", r_frontDistance);
        telemetry.addData("leftFront distance", l_frontDistance);
        telemetry.addData("rightBack distance", r_backDistance);
        telemetry.addData("leftBack distance", l_backDistance);

    }

}

