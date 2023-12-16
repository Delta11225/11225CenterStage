package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoTest extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      waitForStart();

      if (isStopRequested()) return;

      Pose2d startPose = new Pose2d(-36, 61.5, Math.toRadians(180));
      drive.setPoseEstimate(startPose);

      TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)//left spike mark
              .lineTo(new Vector2d(-36, 45))
              .lineToLinearHeading(new Pose2d(-28, 31.5,Math.toRadians(225)))
              .build();

      TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)//center spike mark
              .lineTo(new Vector2d(-36, 25.5))
              .build();

      TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)//right spike mark
              .lineTo(new Vector2d(-36, 45))
              .lineToLinearHeading(new Pose2d(-42.5, 31.5,Math.toRadians(135)))
              .build();

      drive.followTrajectorySequence(traj3);
   }
}
