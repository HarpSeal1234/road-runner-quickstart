package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_UP;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_UP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Non-RR imports


public class Avocado {
    private Servo blockerR;
    private Servo blockerL;
    private DcMotorEx launcher;
    // LEFT BLOCKER

    // RIGHT BLOCKER
    public Avocado(HardwareMap hardwareMap){
        blockerR = hardwareMap.get(Servo.class, "blockerR");
        blockerL = hardwareMap.get(Servo.class, "blockerL");
        launcher = hardwareMap.get(DcMotorEx.class, "outtake1");
        blockerR.setPosition(R_BLOCKER_DOWN);
        blockerL.setPosition(L_BLOCKER_DOWN);
    }

    public class L_Engaged implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            blockerL.setPosition(L_BLOCKER_UP);
            return false;
        }

    }
    public Action l_Engaged(){
        return new L_Engaged();
    }

    public class L_Disengaged implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            blockerL.setPosition(L_BLOCKER_DOWN);
            return false;
        }

    }
    public Action l_Disengaged(){
        return new L_Disengaged();
    }

    public class L_Engaged_Speed_Check implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(launcher.getVelocity() > 1400) {
                blockerL.setPosition(L_BLOCKER_UP);
                return false;
            }
            return true;
        }
    }
    public Action l_Engaged_Speed_Check(){
        return new L_Engaged_Speed_Check();
    }

    public class R_Engaged implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            blockerR.setPosition(R_BLOCKER_UP);
            return false;
        }

    }
    public Action r_Engaged(){
        return new R_Engaged();
    }

    public class R_Disengaged implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            blockerR.setPosition(R_BLOCKER_DOWN);
            return false;
        }

    }
    public Action r_Disengaged(){
        return new R_Disengaged();
    }

    public class R_Engaged_Speed_Check implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(launcher.getVelocity() > 1400) {
                blockerR.setPosition(R_BLOCKER_UP);
                return false;
            }
            return true;
        }
    }

    public Action r_Engaged_Speed_Check(){
        return new R_Engaged_Speed_Check();
    }

}
