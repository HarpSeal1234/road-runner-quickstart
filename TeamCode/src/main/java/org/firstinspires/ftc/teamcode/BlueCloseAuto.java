package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

@Config
@Autonomous(name = "BlueCloseAuto", group = "Autonomous")

public class BlueCloseAuto extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;
    public final static int FAR_OUTTAKE_VELOCITY = 1700;
    public final static int CLOSE_OUTTAKE_VELOCITY = 1200;
    public final static int MEDIUM_OUTTAKE_VELOCITY = 1100;

    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
        Avocado blockerR = new Avocado(hardwareMap);
        Avocado blockerL = new Avocado(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);


        int visionOutputPosition = 1;

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
//                .splineToConstantHeading(new Vector2d(-2,-2),0)
                .splineTo(new Vector2d(-24,-24),Math.toRadians(340));
        waitForStart();
        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh().turn(Math.toRadians(67));

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = path1.build();
        Actions.runBlocking(new ParallelAction(trajectoryActionChosen, outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY), aprilTagDetector.detectAprilTag()));
        int aprilTagId = aprilTagDetector.getDesiredTagId();
        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.update();
        Action trajectoryActionChosen2;

        trajectoryActionChosen2 = path2.build();

        TrajectoryActionBuilder path3 = path2.endTrajectory()
                .fresh()
                .strafeTo(new Vector2d(-10, -43))
//                .strafeTo(new Vector2d(-12.0, -48))
                .turn(Math.toRadians(45))
                .lineToY(-7, new TranslationalVelConstraint(20.0))
                .waitSeconds(0.3)
                .lineToY(-26)
//                .lineToY(-24)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-45))
                .waitSeconds(0.3);
//        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh().strafeTo(new Vector2d(-12.0, -48)).turn(Math.toRadians(45)).lineToY(0.0, new TranslationalVelConstraint(20.0)).waitSeconds(0.3).turn(Math.toRadians(-45)).strafeTo(new Vector2d(-24, -24)).waitSeconds(0.3);
        Action trajectoryActionChosen3;
        trajectoryActionChosen3 = path3.build();

        if(aprilTagId == 23) { // Purple Purple Green
//        sleep(3000);
            Actions.runBlocking(
                    new SequentialAction(
                            (Action) new ParallelAction(trajectoryActionChosen2, intake1.intakeOn()),
                            new SleepAction(0.3),
                            blockerR.r_Engaged(), // purplse
                            new SleepAction(0.3),
                            blockerR.r_Disengaged(),
                            new SleepAction(0.8),
                            blockerR.r_Engaged(), // purple
                            new SleepAction(0.3),
                            new ParallelAction(blockerR.r_Disengaged(), blockerL.l_Engaged()), // green
                            new SleepAction(0.3),
                            new ParallelAction(blockerL.l_Disengaged()),
                            trajectoryActionChosen3,
                            new ParallelAction(blockerL.l_Engaged(), blockerR.r_Engaged()),
                             new SleepAction(0.5),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Disengaged()),
                          new SleepAction(0.7),
                            new ParallelAction(blockerL.l_Engaged(), blockerR.r_Engaged()),
                            new SleepAction(0.5),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Disengaged()),
                            new SleepAction(1.0)

                    )
            );
        } else if (aprilTagId == 22) { // Purple Green Purple
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(trajectoryActionChosen2, intake1.intakeOn()),
                            blockerR.r_Engaged(), // purple #1
                            new SleepAction(0.3),
                            new ParallelAction(blockerR.r_Disengaged(), blockerL.l_Engaged()), // green #1
                            new SleepAction(0.3),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Engaged()), // purple #2
                            new SleepAction(0.3),
                            blockerR.r_Disengaged(),
                            trajectoryActionChosen3,
                            new ParallelAction(blockerL.l_Engaged(), blockerR.r_Engaged()),
                            new SleepAction(0.5),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Disengaged()),
                            new SleepAction(0.7),
                            new ParallelAction(blockerL.l_Engaged(), blockerR.r_Engaged()),
                            new SleepAction(0.5),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Disengaged()),
                            new SleepAction(1.0)
                    )
            );
        }else if ( aprilTagId == 21){ //Green Purple Purple
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(trajectoryActionChosen2, intake1.intakeOn()),
                            blockerL.l_Engaged(), // green ball #1 start
                            new SleepAction(0.3),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Engaged()), // green ball #1 end // purple ball #1 start
                            new SleepAction(0.3),
                            blockerR.r_Disengaged(), // purple ball #1 end
                            new SleepAction(0.7),
                            blockerR.r_Engaged(), // purple #2 start
                            new SleepAction(0.5),
                            blockerR.r_Disengaged(),
                            trajectoryActionChosen3,
                            pivot.closePivot(),
                            outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY),
                            new SleepAction(0.3),
                            new ParallelAction(blockerL.l_Engaged(), blockerR.r_Engaged()),
                            new SleepAction(0.5),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Disengaged()),
                            new SleepAction(0.9),
                            new ParallelAction(blockerL.l_Engaged(), blockerR.r_Engaged()),
                            new SleepAction(0.5),
                            new ParallelAction(blockerL.l_Disengaged(), blockerR.r_Disengaged()),
                            new SleepAction(1.0)
                    )
            );
        }
    }

    }




