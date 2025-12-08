package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="blinkin", group="Linear OpMode")
@Disabled

public class BlinkinTest extends LinearOpMode{
    RevBlinkinLedDriver blinkin;
//    RevBlinkinLedDriver leftLED;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern;
//    RevBlinkinLedDriver.BlinkinPattern L_pattern;
    @Override
    public void runOpMode() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkin.setPattern(blinkinPattern);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            } else if (gamepad1.b) {
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else if (gamepad1.x) {
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            } else if (gamepad1.y){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            } else if (gamepad1.right_bumper){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
            }
            else if (gamepad1.left_bumper){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
            }
            else if (gamepad1.dpad_up){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND;
            }
            else if (gamepad1.dpad_down){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2;

            }
            else if (gamepad1.dpad_right){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST;

            }
            else if (gamepad1.dpad_left){
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM;

            }
            blinkin.setPattern(blinkinPattern);

        }
    }


}
