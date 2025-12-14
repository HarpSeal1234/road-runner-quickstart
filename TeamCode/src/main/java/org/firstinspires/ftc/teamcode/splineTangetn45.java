package org.firstinspires.ftc.teamcode;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Disabled
@Autonomous(name = "spline 45", group = "Autonomous")

public class splineTangetn45 extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive ;
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();
        if (isStopRequested()) return;

        // TURN TO SHOOT
        TrajectoryActionBuilder path2 = drive.actionBuilder(initialPose)
                .fresh()
                .splineToLinearHeading(new Pose2d(-24,24,Math.toRadians(5)),Math.toRadians(45));
        Action trajectoryActionChosen2 = path2.build();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen2
                )
        );
    }
}
