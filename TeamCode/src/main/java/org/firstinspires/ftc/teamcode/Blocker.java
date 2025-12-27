package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.BLOCKER_CLOSE;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.BLOCKER_OPEN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.L_BLOCKER_UP;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_DOWN;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.R_BLOCKER_UP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Non-RR imports

public class Blocker {
    private Servo blocker;
    private DcMotorEx launcher;
    // LEFT BLOCKER

    // RIGHT BLOCKER
    public Blocker(HardwareMap hardwareMap){
        blocker = hardwareMap.get(Servo.class, "blocker");
        launcher = hardwareMap.get(DcMotorEx.class, "outtake1");
        blocker.setPosition(R_BLOCKER_DOWN);
    }

    public class BlockerOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            blocker.setPosition(BLOCKER_OPEN);
            return false;
        }

    }
    public Action blockerOpen(){
        return new BlockerOpen();
    }

    public class BlockerClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            blocker.setPosition(BLOCKER_CLOSE);
            return false;
        }

    }
    public Action blockerClose(){
        return new BlockerClose();
    }

    public class BlockerOpenSpeedCheck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(launcher.getVelocity() > 1400) {
                blocker.setPosition(BLOCKER_OPEN);
                return false;
            }
            return true;
        }
    }
    public Action blockerOpenSpeedCheck(){
        return new BlockerOpenSpeedCheck();
    }

}
