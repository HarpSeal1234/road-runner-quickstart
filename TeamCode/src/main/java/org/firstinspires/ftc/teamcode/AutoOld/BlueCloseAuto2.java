package org.firstinspires.ftc.teamcode.AutoOld;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_OUTTAKE_VELOCITY;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Avocado;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pivot;

@Config
@Disabled
@Autonomous(name = "BlueCloseAuto2", group = "Autonomous")

public class BlueCloseAuto2 extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive ;
    public void reportPosition(){
        telemetry.addData("Current Position", this.drive.localizer.getPose());
        telemetry.update();
    }
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);
        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
        Avocado blockerR = new Avocado(hardwareMap);
        Avocado blockerL = new Avocado(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);


        int visionOutputPosition = 1;

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-24, -36), Math.toRadians(0));
        pivot.closePivot();
//        reportPosition();

        waitForStart();
        if (isStopRequested()) return;

        Action trajectoryActionChosen1 = path1.build();
        Actions.runBlocking(new ParallelAction(outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+200), trajectoryActionChosen1,aprilTagDetector.detectAprilTag()));
        int aprilTagId = aprilTagDetector.getDesiredTagId();
        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.update();
        TrajectoryActionBuilder path2 = path1.endTrajectory()
                .fresh()
                .turn(Math.toRadians(45));

        if (isStopRequested()) return;

        Action trajectoryActionChosen2 = path2.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen2,
                        new ParallelAction(blockerL.r_Engaged(), blockerR.l_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.2),
                        new ParallelAction(blockerL.r_Disengaged(), blockerR.l_Disengaged()), // purple ball #1 end
                        new SleepAction(0.5),
                        intake1.intakeOn(),
                        new ParallelAction(blockerL.r_Engaged(), blockerR.l_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.8)
                )
        );
        TrajectoryActionBuilder path3 = path2.endTrajectory()
                .fresh()
                .splineTo(new Vector2d(-36.0, 0.0), Math.toRadians(100))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-24.0, -36), Math.toRadians(48));
    }
}
