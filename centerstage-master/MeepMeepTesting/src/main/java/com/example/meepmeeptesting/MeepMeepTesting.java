package com.example.meepmeeptesting;

import static queue.dicelibrary.Objectives.PURPLE_YELLOW;
import static queue.dicelibrary.Objectives.PURPLE_YELLOW_WHITE;
import static queue.dicelibrary.RobotRoutes.MAXIMUM_ACCELERATION;
import static queue.dicelibrary.RobotRoutes.MAXIMUM_ANGULAR_ACCELERATION;
import static queue.dicelibrary.RobotRoutes.MAXIMUM_ANGULAR_VELOCITY;
import static queue.dicelibrary.RobotRoutes.MAXIMUM_VELOCITY_NORMAL;
import static queue.dicelibrary.RobotRoutes.MAXIMUM_VELOCITY_SLOW;
import static queue.dicelibrary.RobotRoutes.TRACK_WIDTH;
import static queue.dicelibrary.RobotRoutes.driveToBackdropApproach;
import static queue.dicelibrary.RobotRoutes.driveToBackdropPlace;
import static queue.dicelibrary.RobotRoutes.driveToSpikeMark;
import static queue.dicelibrary.RobotRoutes.driveToStackApproach;
import static queue.dicelibrary.RobotRoutes.driveToStackGrab;
import static queue.dicelibrary.RobotRoutes.getDefaultGrabStackX;
import static queue.dicelibrary.RobotRoutes.getDefaultGrabStackY;
import static queue.dicelibrary.RobotRoutes.getDefaultPlaceBackdropX;
import static queue.dicelibrary.RobotRoutes.getDefaultPlaceBackdropY;
import static queue.dicelibrary.RobotRoutes.getMaximumVelocityFast;
import static queue.dicelibrary.RobotRoutes.getStartPose;
import static queue.dicelibrary.RobotRoutes.park;
import static queue.dicelibrary.RobotRoutes.returnToBackdrop;
import static queue.dicelibrary.TeamPropLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

import queue.dicelibrary.Action;
import queue.dicelibrary.BackAction;
import queue.dicelibrary.LineToAction;
import queue.dicelibrary.LineToLinearHeadingAction;
import queue.dicelibrary.Objectives;
import queue.dicelibrary.RobotPose;
import queue.dicelibrary.SetReversedAction;
import queue.dicelibrary.SetTangentAction;
import queue.dicelibrary.SplineToAction;
import queue.dicelibrary.SplineToConstantHeadingAction;
import queue.dicelibrary.SplineToLinearHeadingAction;
import queue.dicelibrary.TeamPropLocation;
import queue.dicelibrary.TurnAction;

public class MeepMeepTesting {

    private static final boolean RED_ALLIANCE = true;
    private static final boolean START_CLOSE = false;
    private static final TeamPropLocation LOCATION = RIGHT;
    private static final boolean PARK_LEFT = false;
    private static final Objectives objectives = PURPLE_YELLOW_WHITE;

    // Runs the application.
    public static void main(String[] args) throws Exception {

        // Get a MeepMeep interface.
        MeepMeep meepMeep = new MeepMeep(800);

        // Get a robot.
        RoadRunnerBotEntity robot = getRobot(meepMeep);

        // Start the simulation.
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();

    }

    // Gets a robot.
    private static RoadRunnerBotEntity getRobot(MeepMeep meepMeep) throws Exception {

        // Get a start pose.
        RobotPose inputStartPose = getStartPose(RED_ALLIANCE, START_CLOSE);
        Pose2d outputStartPose = new Pose2d(inputStartPose.x, inputStartPose.y, inputStartPose.heading);

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(MAXIMUM_VELOCITY_NORMAL, TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = new TrajectorySequenceBuilder(outputStartPose, null, velocityConstraint, accelerationConstraint, MAXIMUM_ANGULAR_VELOCITY, MAXIMUM_ANGULAR_ACCELERATION);

        // Get a fast maximum velocity.
        double maximumVelocityFast = getMaximumVelocityFast(RED_ALLIANCE, START_CLOSE, LOCATION);

        // Drive to the spike mark.
        applyActions(driveToSpikeMark(RED_ALLIANCE, START_CLOSE, LOCATION), trajectorySequenceBuilder, maximumVelocityFast);

        // Wait for a bit.
        trajectorySequenceBuilder.waitSeconds(1);

        // If we are placing the yellow pixel...
        if(objectives == PURPLE_YELLOW || objectives == PURPLE_YELLOW_WHITE) {

            double grabStackX = getDefaultGrabStackX(RED_ALLIANCE);
            double grabStackY = getDefaultGrabStackY(RED_ALLIANCE);
            double placeBackdropX = getDefaultPlaceBackdropX(RED_ALLIANCE);
            double placeBackdropY = getDefaultPlaceBackdropY(RED_ALLIANCE);

            // Drive to the backdrop approach position.
            applyActions(driveToBackdropApproach(RED_ALLIANCE, START_CLOSE, LOCATION, placeBackdropX, placeBackdropY, grabStackY), trajectorySequenceBuilder, maximumVelocityFast);

            // Wait for a bit.
            trajectorySequenceBuilder.waitSeconds(1);

            // Drive to the backdrop place position.
            applyActions(driveToBackdropPlace(RED_ALLIANCE, LOCATION, true, placeBackdropX, placeBackdropY), trajectorySequenceBuilder, MAXIMUM_VELOCITY_SLOW);

            // Wait for a bit.
            trajectorySequenceBuilder.waitSeconds(1);

            // If we are placing white pixels...
            if(objectives == PURPLE_YELLOW_WHITE) {

                // Drive to the stack approach position.
                applyActions(driveToStackApproach(RED_ALLIANCE, grabStackX, grabStackY), trajectorySequenceBuilder, maximumVelocityFast);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

                // Drive to the stack grab position.
                applyActions(driveToStackGrab(RED_ALLIANCE, grabStackX, grabStackY), trajectorySequenceBuilder, MAXIMUM_VELOCITY_SLOW);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(2);

                // Return to the backdrop.
                applyActions(returnToBackdrop(RED_ALLIANCE, placeBackdropX, placeBackdropY, grabStackY), trajectorySequenceBuilder, maximumVelocityFast);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

                // Drive to the backdrop place position.
                applyActions(driveToBackdropPlace(RED_ALLIANCE, LOCATION, false, placeBackdropX, placeBackdropY), trajectorySequenceBuilder, MAXIMUM_VELOCITY_SLOW);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

            }

        }

        // If we should park...
        if(START_CLOSE || objectives == PURPLE_YELLOW || objectives == PURPLE_YELLOW_WHITE) {

            // Park.
            applyActions(park(RED_ALLIANCE, PARK_LEFT), trajectorySequenceBuilder, maximumVelocityFast);

        }

        // Build a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Construct a robot.
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep).followTrajectorySequence(trajectorySequence);

        // Return the robot.
        return robot;

    }

    public static void applyActions(List<Action> actions, TrajectorySequenceBuilder trajectorySequenceBuilder, double maximumVelocity) throws Exception {

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(maximumVelocity, TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Apply the constraints.
        trajectorySequenceBuilder.setConstraints(velocityConstraint, accelerationConstraint);

        // Apply the actions.
        for(Action inputAction : actions) {
            if(inputAction instanceof BackAction) {
                BackAction outputAction = (BackAction)inputAction;
                trajectorySequenceBuilder.back(outputAction.distance);
            }
            else if(inputAction instanceof LineToAction) {
                LineToAction outputAction = (LineToAction)inputAction;
                trajectorySequenceBuilder.lineTo(new Vector2d(outputAction.x, outputAction.y));
            }
            else if(inputAction instanceof LineToLinearHeadingAction) {
                LineToLinearHeadingAction outputAction = (LineToLinearHeadingAction)inputAction;
                trajectorySequenceBuilder.lineToLinearHeading(new Pose2d(outputAction.x, outputAction.y, outputAction.heading));
            }
            else if(inputAction instanceof SetReversedAction) {
                SetReversedAction outputAction = (SetReversedAction)inputAction;
                trajectorySequenceBuilder.setReversed(outputAction.reversed);
            }
            else if(inputAction instanceof SetTangentAction) {
                SetTangentAction outputAction = (SetTangentAction)inputAction;
                trajectorySequenceBuilder.setTangent(outputAction.tangent);
            }
            else if(inputAction instanceof SplineToAction) {
                SplineToAction outputAction = (SplineToAction)inputAction;
                trajectorySequenceBuilder.splineTo(new Vector2d(outputAction.x, outputAction.y), outputAction.heading);
            }
            else if(inputAction instanceof SplineToConstantHeadingAction) {
                SplineToConstantHeadingAction outputAction = (SplineToConstantHeadingAction)inputAction;
                trajectorySequenceBuilder.splineToConstantHeading(new Vector2d(outputAction.x, outputAction.y), outputAction.heading);
            }
            else if(inputAction instanceof SplineToLinearHeadingAction) {
                SplineToLinearHeadingAction outputAction = (SplineToLinearHeadingAction)inputAction;
                trajectorySequenceBuilder.splineToLinearHeading(new Pose2d(outputAction.x, outputAction.y, outputAction.heading), outputAction.tangent);
            }
            else if(inputAction instanceof TurnAction) {
                TurnAction outputAction = (TurnAction)inputAction;
                trajectorySequenceBuilder.turn(outputAction.angle);
            }
            else {
                throw new Exception("The action type is unrecognized.");
            }
        }

    }

}