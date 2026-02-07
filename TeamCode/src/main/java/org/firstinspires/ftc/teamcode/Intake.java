package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports


public class Intake {
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    public final static double INTAKE_POWER = 0.9;
    public final static double INTAKE_ZERO_POWER = 0.0;
    public Intake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class IntakeOn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake1.setPower(INTAKE_POWER);
            intake2.setPower(-0.5);
            return false;
        }

    }
    public Action intakeOn(){
        return new IntakeOn();
    }
    public class IntakeOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake1.setPower(INTAKE_ZERO_POWER);
            intake2.setPower(0);
            return false;
        }
    }
    public Action intakeOff() {
        return new IntakeOff();
    }
    public class IntakeLaunch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake2.setPower(1.0);
            intake1.setPower(INTAKE_POWER);
            return false;
        }
    }
    public Action intakeLaunch() {
        return new IntakeLaunch();
    }
    public class IntakeOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake1.setPower(-INTAKE_POWER);
            return false;
        }
    }
    public Action intakeOut() {
        return new IntakeOut();
    }
}
