package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.autoBallsin2;
// PEdro pathing is so fucking hard to follow I won't bother with it next year
@Autonomous(name = "Auton blue side", group = "Autonomous")
@Configurable // Panels
public class autoBallsIn3 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    //private autoBallsin2.Paths paths; // Paths defined in the Paths class
    private String launcherState = "End";
    private String pathingState = "";
    private DcMotor leftLaunch;
    private Servo launchTurn;
    private DcMotor rightLaunch;
    private DcMotor liftUp;
    private Servo launchPush;
    private CRServo intake;
    private Servo liftPush;
    private ColorSensor sensorIntake;
    private ColorSensor sensorLift;
    private TouchSensor liftTouch;
    private TouchSensor sensorLiftLimit;
    private boolean runOnce = false;
    private boolean runOnce2 = false;
    private long timeSince1 = 0;
    private long timeSince2 = 0;
    private String liftState = "";
    private boolean launcherFree = false;
    private Timer timer = new Timer();
    private String intakeState = "on";
    private boolean returnToEnd = false;
    private CoordinateSystem coordinateSystem; // fucking shit isn't working

    long waitTime1 = 0;
    String returnState1 = "";
    long timeZero = 0;

    private void BeginWaiting(String returnState, long wait)
    {
        waitTime1 = wait;
        returnState1 = returnState;
        timeZero = System.currentTimeMillis();
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        launchTurn = hardwareMap.get(Servo.class, "launchTurn");
        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");
        launchPush = hardwareMap.get(Servo.class, "launchPush");
        liftUp = hardwareMap.get(DcMotor.class, "liftUp");
        intake = hardwareMap.get(CRServo.class, "intake");
        liftPush = hardwareMap.get(Servo.class, "liftPush");
        sensorIntake = hardwareMap.get(ColorSensor.class, "sensorIntake");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        sensorLift = hardwareMap.get(ColorSensor.class, "sensorLift");
        liftTouch=hardwareMap.get(TouchSensor.class, "liftTouch");
        sensorLiftLimit = hardwareMap.get(TouchSensor.class, "sensorLiftLimit");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.084788029925186, 123.53117206982543, Math.toRadians(315)));


        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Launch State", launcherState);
        panelsTelemetry.debug("Lift State", liftState);
        panelsTelemetry.debug("Path State", pathingState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("launchPush", launchPush.getPosition());
        panelsTelemetry.debug("liftPush", liftPush.getPosition());
        panelsTelemetry.debug("Ball detected Top", sensorLift.green() >=84 || sensorLift.blue() >=84);
        panelsTelemetry.debug("ball detected Bottom", sensorIntake.blue()>=115 || sensorIntake.green()>=115);

        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.084788029925186, 123.53117206982543), new Pose(36.44887780548628, 106.65336658354114))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(315))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathingState) {
            case "":
                follower.followPath(paths.Path1, true); // REMOVE IF BEFORE COMP
                // add check here if follower done
                if (!follower.atParametricEnd()) {pathingState = "atPosition";}
                break;
            case "atPosition":
                launcherState = "ballInLauncher";
                pathingState = "End";
                break;
            case "End":
                break;
        }
        switch (launcherState){
            case "End":
                panelsTelemetry.update(telemetry);
                break;
            case "ballInLauncher":
                if (!runOnce) {
                    ((DcMotorEx) leftLaunch).setVelocity(-1500 * 0.6);
                    ((DcMotorEx) rightLaunch).setVelocity(1500* 0.6);
                    timeSince1 = System.currentTimeMillis();
                    runOnce = true;
                }
                if (System.currentTimeMillis() >= timeSince1 + 2000) {

                    if (!runOnce2){
                        launchPush.setPosition(1);
                        timeSince2 = System.currentTimeMillis();
                        runOnce2 = true;
                    }
                    if (System.currentTimeMillis() >= timeSince2 + 2000){
                        //launchPush.setPosition(0.5);
                        ((DcMotorEx) leftLaunch).setVelocity(0);
                        ((DcMotorEx) rightLaunch).setVelocity(0);
                        launcherFree = true;
                        launcherState = "End";
                        liftState = "upBall";
                        //terminateOpModeNow();
                    }


                }
                break;
            case "Wait":
                if (System.currentTimeMillis() >= waitTime1 + timeZero) {
                    launcherState = returnState1;
                }
                break;
            case "gotoBallInLauncher":
                runOnce = false;
                runOnce2 = false;
                launcherState = "ballInLauncher";
                break;
        }
        switch (liftState) {
            case "": break;
            case "upBall":
                if (!(sensorLift.green() >=84 || sensorLift.blue() >=84)) { //if no ball
                    // No ball to push, so we're done
                    liftState = "upBall2";

                    break;
                } else if (returnToEnd) {liftState = ""; launcherState=""; break;}
                launchPush.setPosition(0.4);
                liftPush.setPosition(0);
                launchTurn.setPosition(0.5);
                launcherFree = false;
                BeginWaiting("upBall2", 2000);
                liftState = "Wait";
                //liftState = "upBall2";
                break;
            case "upBall2":
                liftPush.setPosition(0.7);
                if (!launcherFree && (sensorLift.green() >=84 || sensorLift.blue() >=84)) { // if launcher free + ball
                    liftState = "upBall3"; // launch
                } else if (sensorIntake.blue()>=115 || sensorIntake.green()>=115) {
                    liftState = "goDown-thenUp";
                } else if (launcherFree && (sensorLift.green() >=84 || sensorLift.blue()>=84)){
                    liftState = "upBall";
                } else {
                    liftState="upBall3-2";
                    launcherState = "End";
                }
                break;
            case "upBall3":
                launcherState = "gotoBallInLauncher";
                liftState = "";
                break;
            case "upBall3-2":
                launcherState = "gotoBallInLauncher";
                returnToEnd=true;
                liftState = "";
                break;
            case "goDown-thenUp":
                while (!liftTouch.isPressed()) {
                    launchPush.setPosition(0.5);
                    ((DcMotorEx) liftUp).setVelocity(250);
                }
                ((DcMotorEx) liftUp).setVelocity(0);
                liftState = "goUp-thenPush";
            case "goUp-thenPush":
                while (!sensorLiftLimit.isPressed()) {//Lift until at launch height

                    launchPush.setPosition(0.5);
                    ((DcMotorEx) liftUp).setVelocity(-250);
                }
                ((DcMotorEx) liftUp).setVelocity(0); // now carrying ball
                liftState = "upBall";
            case "Wait":
                if (System.currentTimeMillis() >= waitTime1 + timeZero) {
                    liftState = returnState1;
                }
                break;
        }
        switch (intakeState) {
            case "":
                break;
            case "on":
                intake.setPower(-1.0);
                intakeState = ""; // Set to "" to prevent running again
                break;
            case "off":
                intake.setPower(0);
                intakeState = ""; // Set to "" to prevent running again
                break;
            case "Wait":
                if (System.currentTimeMillis() >= waitTime1 + timeZero) {
                    intakeState = returnState1;
                }
                break;
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
