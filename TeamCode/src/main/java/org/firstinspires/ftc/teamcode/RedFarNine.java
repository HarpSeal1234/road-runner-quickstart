package org.firstinspires.ftc.teamcode;

// RR-specific imports

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.trajectoryWait;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "Red Far Auto", group = "Autonomous")
public class RedFarNine extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive;

    public void reportPosition() {
        telemetry.addData("Current Position", this.drive.localizer.getPose());
        telemetry.update();
    }

    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);
//        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
//        Avocado blocker = new Avocado(hardwareMap);
//        Pivot pivot = new Pivot(hardwareMap);
//        BallManager ballManager = new BallManager(hardwareMap);


//        pivot.farPivot();


        waitForStart();
        if (isStopRequested()) return;

//        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.update();

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(9.5, 2), Math.toRadians(-19.5)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait);
        Action trajectoryActionChosen1 = path1.build();

            Actions.runBlocking(
                    new SequentialAction(
                            outtake1.startLauncher(1750),
                            trajectoryActionChosen1,
                            new SleepAction(0.05),
                            intake1.intakeLaunch(),
                            new SleepAction(1.5),
                            intake1.intakeOff()
                    )
            );

        TrajectoryActionBuilder path2 = path1.endTrajectory()
                .fresh()
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(26, -16), Math.toRadians(-90)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait)
                .lineToY(-30, new TranslationalVelConstraint(17.0))
                .waitSeconds(trajectoryWait);
        Action trajectoryActionChosen2 = path2.build();

        TrajectoryActionBuilder toShooterP1 = path2.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(9, -1), Math.toRadians(-22)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait);
        Action toShooterA1 = toShooterP1.build();

        Actions.runBlocking(
                new SequentialAction(
                        intake1.intakeOn(),
                        trajectoryActionChosen2,
                        outtake1.startLauncher(1600),
                        intake1.intakeOff(),
                        new SleepAction(0.05),
                        toShooterA1,
                        intake1.intakeLaunch(),
                        new SleepAction(1.5),
                        intake1.intakeOff()
                )
        );

        TrajectoryActionBuilder path3 = toShooterP1.endTrajectory()
                .fresh()
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(3.5, -24), Math.toRadians(-95)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait+0.2)
                .lineToY(-45, new TranslationalVelConstraint(15))
                .waitSeconds(trajectoryWait)
                .strafeTo(new Vector2d(9,-45));
        Action trajectoryActionChosen3 = path3.build();

        TrajectoryActionBuilder toShooterP2 = path3.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(9.5, -1), Math.toRadians(335)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait);
        Action toShooterA2 = toShooterP2.build();



        TrajectoryActionBuilder leavePath = toShooterP2.endTrajectory()
                .fresh()
                .splineToConstantHeading(new Vector2d(10,-10),0)
                .waitSeconds(trajectoryWait);
        Action leave = leavePath.build();

        Actions.runBlocking(
                new SequentialAction(
                        intake1.intakeOn(),
                        outtake1.startLauncher(1500),
                        trajectoryActionChosen3,
                        intake1.intakeOff(),
                        new SleepAction(0.05),
                        toShooterA2,
                        new SleepAction(0.05),
                        intake1.intakeLaunch(),
                        new SleepAction(1.5),
                        leave
                )
        );
    }
}