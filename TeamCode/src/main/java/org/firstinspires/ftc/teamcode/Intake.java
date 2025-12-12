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
//    private CRServo pushR;
//    private CRServo pushL;
    public final static double INTAKE_POWER = 0.9;
    public final static double INTAKE_ZERO_POWER = 0.0;
    public Intake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setPower(0.0);
    }

    public class IntakeOn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake1.setPower(INTAKE_POWER);
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
            return false;
        }
    }
    public Action intakeOff() {
        return new IntakeOff();
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
