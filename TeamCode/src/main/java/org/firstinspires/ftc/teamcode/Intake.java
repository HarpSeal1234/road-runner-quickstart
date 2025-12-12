package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

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
    public Action IntakeOff() {
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
