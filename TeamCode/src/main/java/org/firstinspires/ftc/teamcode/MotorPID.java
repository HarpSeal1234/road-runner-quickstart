package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MotorPID")
public class MotorPID extends LinearOpMode {
//Motor Variables
    private DcMotorEx motorOne;
    private double motorOneZeroPower = 0.0;
    private double motorOneMaxPower = 1.0;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneTargetVelocity = 1100;
    private double motorOneMaxVelocity = 2800;

//PID Variables
    private double F = 32767/motorOneMaxVelocity;
    private double kP = 1.6;
    private double kI = kP * 0.1;
    private double kD = kP * 0.01;
    private double position = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            motorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()) {
            runMotorOne(motorOneTargetVelocity);
            motorTelemetry();
        }
    }

    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
    }
//initialize motorOne
    public void initMotorOne(double kP, double kI, double kD, double F, double position) {
        motorOne = hardwareMap.get(DcMotorEx.class, "outtake1");
        motorOne.setDirection(DcMotor.Direction.FORWARD);
        motorOne.setPower(motorOneZeroPower);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorOne.setVelocityPIDFCoefficients(kP, kI, kD, F);
        motorOne.setPositionPIDFCoefficients(position);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runMotorOne(double velocity) {
        motorOne.setVelocity(velocity);
        motorOneCurrentVelocity = motorOne.getVelocity();
        if (motorOneCurrentVelocity > motorOneMaxVelocity) {
            motorOneMaxVelocity = motorOneCurrentVelocity;
        }
    }

    public void motorTelemetry() {
        telemetry.log().clear();
        telemetry.addData("Power", motorOne.getPower());
        telemetry.addData("Target Velocity", motorOneTargetVelocity);
        telemetry.addData("Max Velocity", motorOneMaxVelocity);
        telemetry.addData("Current Velocity", motorOneCurrentVelocity);
        telemetry.addData("F", F);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD","%.5f", kD);
        telemetry.update();
    }
}
