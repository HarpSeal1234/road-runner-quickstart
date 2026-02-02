package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kD;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kI;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kP;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Launcher {
    private DcMotorEx outtake1 = null;
    private DcMotorEx outtake2 = null;
    private Servo pivot;
    // OUTTAKE
    /*
    public final static double FAR_OUTTAKE_VELOCITY = 1700;
    public final static double CLOSE_OUTTAKE_VELOCITY = 1300;*/
    private double outtakeMaxVelocity = 2800;
    private double F = 32767/outtakeMaxVelocity;
//    private double kP = 1.25;
//    private double kI = kP * 0.1;
//    private double kD = kP * 0.01;
    private double position = 5.0;
    public Launcher(HardwareMap hardwareMap){
        outtake1 = hardwareMap.get(DcMotorEx.class,"outtake1");
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake1.setVelocity(0.0);

        outtake2  = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(0.0);
    }

    public class StartLauncher implements Action {
        private int velocity;
        public StartLauncher(int velocity){
            this.velocity = velocity;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake1.setVelocity(velocity);
            outtake1.setVelocityPIDFCoefficients(2.8,0,0,16.2);
            outtake2.setVelocity(velocity);
            outtake2.setVelocityPIDFCoefficients(2.8,0,0,16.2);
            return false;
        }

    }
    public Action startLauncher(int velocity){
        return new StartLauncher(velocity);
    }
    /*
    public class LauncherFar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake1.setVelocity(1500);
            outtake2.setVelocity(1500);
            outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(0.4,0,0,15.7));
            outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(0.4,0,0,15.7));
            return false;
        }

    }
    public Action launcherFar(){
        return new LauncherFar();
    }
*/
    public class LauncherOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake1.setVelocity(0.0);
            outtake2.setVelocity(0.0);
            return false;
        }

    }
    public Action launcherOff(){
        return new LauncherOff();
    }

}

