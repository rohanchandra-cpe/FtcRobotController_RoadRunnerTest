package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(group = "drive")

public class MyOpmode extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose=new Pose2d(24, 24, Math.toRadians(0.00));

        drive.setPoseEstimate(startPose);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(24, 48), Math.toRadians(0.0))
                .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(0.0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(untitled0);
    }
}
