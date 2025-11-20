package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.Build.VERSION_CODES.R;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Hashtable;

@TeleOp(name = "mechanism2Gamepad")
public class mechanism2Gamepad extends LinearOpMode {

    private Servo launchTurn;
    private DcMotor leftLaunch;
    private DcMotor rightLaunch;
    private Servo launchPush;
    private DcMotor liftUp;
    private CRServo intake;
    private Servo liftPush;
    private ColorSensor sensorLauncher;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private TouchSensor sensorLiftLimit;
    private TouchSensor liftTouch;
    private ColorSensor sensorIntake;
    private ColorSensor sensorLift;
    private long timeSince4 = 0;
    private boolean runOnce = false;
    private boolean runOnce2 = false;
    private boolean runOnce3 = false;
    private String liftState = "Unconfigured";
    private boolean launcherEmpty;
    private Hashtable<String, Boolean> states = new Hashtable<>();
    private String launcherState = "";

    long waitTime1 = 0;
    String returnState1 = "";
    long timeZero = 0;
    double servoPos1 = 0.0;
    private int driveGamepad = 2; // DO NOT CHANGE

    private void BeginWaiting(String returnState, long wait)
    {
        waitTime1 = wait;
        returnState1 = returnState;
        timeZero = System.currentTimeMillis();
    }
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     *
     *
     */
    @Override
    public void runOpMode() {
        double configSpeedInt = 0.5;
        boolean motorMenuActivated;
        String configSpeedTxt;
        long timeSince3;
        long timeSince;
        long timeSinceMotorSpeedMenuActivate;
        long timeSince2 = 0;
        long timeSince6 = 0;
        boolean intakeMode = false;
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        launchTurn = hardwareMap.get(Servo.class, "launchTurn");
        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");
        launchPush = hardwareMap.get(Servo.class, "launchPush");
        liftUp = hardwareMap.get(DcMotor.class, "liftUp");
        intake = hardwareMap.get(CRServo.class, "intake");
        liftPush = hardwareMap.get(Servo.class, "liftPush");
        sensorLiftLimit = hardwareMap.get(TouchSensor.class, "sensorLiftLimit");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");
        sensorIntake = hardwareMap.get(ColorSensor.class, "sensorIntake");
        sensorLift = hardwareMap.get(ColorSensor.class, "sensorLift");
        //sensorLauncher = hardwareMap.get(ColorSensor.class, "sensorLauncher");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        // Put initialization blocks here.
        launchTurn.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        configSpeedTxt = "What speed? Press B to confirm and use dpad up/down to select";
        configSpeedInt = 1;
        if (opModeIsActive()) {
            // Put run blocks here.
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            timeSince3 = System.currentTimeMillis();
            while (opModeIsActive()) {
                Drive();
                // Put loop blocks here.
                telemetry.update();
                // Get the current time in milliseconds. The value returned represents
                // the number of milliseconds since midnight, January 1, 1970 UTC.
                timeSince = System.currentTimeMillis();
                if (gamepad1.b) {
                    motorMenuActivated = true;
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    timeSinceMotorSpeedMenuActivate = System.currentTimeMillis();
                    while (opModeIsActive()) {
                        telemetry.addData(configSpeedTxt, configSpeedInt);
                        telemetry.addData("Tip", "Press X to exit");
                        // Get the current time in milliseconds. The value returned represents
                        // the number of milliseconds since midnight, January 1, 1970 UTC.
                        // Get the current time in milliseconds. The value returned represents
                        // the number of milliseconds since midnight, January 1, 1970 UTC.
                        if (gamepad1.dpad_up && System.currentTimeMillis() >= timeSince + 200 && !(configSpeedInt >= 1)) {
                            // Get the current time in milliseconds. The value returned represents
                            // the number of milliseconds since midnight, January 1, 1970 UTC.
                            timeSince = System.currentTimeMillis();
                            configSpeedInt += 0.1;
                        } else if (gamepad1.dpad_down && System.currentTimeMillis() >= timeSince + 200 && !(configSpeedInt <= -1)) {
                            // Get the current time in milliseconds. The value returned represents
                            // the number of milliseconds since midnight, January 1, 1970 UTC.
                            timeSince = System.currentTimeMillis();
                            configSpeedInt -= 0.1;
                        }
                        // Get the current time in milliseconds. The value returned represents
                        // the number of milliseconds since midnight, January 1, 1970 UTC.
                        if (gamepad1.b && System.currentTimeMillis() >= timeSinceMotorSpeedMenuActivate + 500) {
                            while (true) {
                                ((DcMotorEx) leftLaunch).setVelocity(-configSpeedInt * 1500);
                                ((DcMotorEx) rightLaunch).setVelocity(configSpeedInt * 1500);
                                telemetry.addData("Launcher angle", launchTurn.getPosition());
                                if (gamepad1.right_bumper) {
                                    sleep(1000);
                                    // 1 is pushed to launch, 0.5 is open for pushing into launcher
                                    launchPush.setPosition(1);
                                    sleep(700);
                                    launchPush.setPosition(0.5);
                                } else if (gamepad1.x) {
                                    leftLaunch.setPower(0);
                                    rightLaunch.setPower(0);
                                    break;
                                }
                                while (gamepad1.dpad_right) {
                                    launchPush.setPosition(0.4);

                                    liftPush.setPosition(0);

                                }

                                if (gamepad1.a && System.currentTimeMillis() >= 300 + timeSince3) {
                                    if (servoPos1 < 0) {servoPos1 = 0.0;}
                                    // Get the current time in milliseconds. The value returned represents
                                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                                    timeSince3 = System.currentTimeMillis();
                                    launchTurn.setPosition(servoPos1 + 0.1);// TODO: CHANGE ME
                                    servoPos1 +=0.1;
                                    // Get the current time in milliseconds. The value returned represents
                                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                                    timeSince2 = System.currentTimeMillis();
                                } else if (gamepad1.y && System.currentTimeMillis() >= 300 + timeSince3) {
                                    if (servoPos1 < 0) {servoPos1 = 0.0;}
                                    // Get the current time in milliseconds. The value returned represents
                                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                                    timeSince3 = System.currentTimeMillis();
                                    launchTurn.setPosition(servoPos1 - 0.1); // TODO: CHANGE ME
                                    servoPos1 -=0.1;
                                    // Get the current time in milliseconds. The value returned represents
                                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                                    timeSince2 = System.currentTimeMillis();
                                }
                                if (System.currentTimeMillis() >= 150 + timeSince2) {
                                    telemetry.update();
                                }
                                while (gamepad1.dpad_up) {
                                    if (!sensorLiftLimit.isPressed()) {
                                        launchPush.setPosition(0.5);
                                        ((DcMotorEx) liftUp).setVelocity(-250);
                                    } else {
                                        ((DcMotorEx) liftUp).setVelocity(0);
                                    }
                                }
                                ((DcMotorEx) liftUp).setVelocity(0);
                                while (gamepad1.dpad_down && !liftTouch.isPressed()) {
                                    launchPush.setPosition(0.5);
                                    ((DcMotorEx) liftUp).setVelocity(250);
                                }
                                ((DcMotorEx) liftUp).setVelocity(0);
                                if (gamepad1.dpad_left) {
                                    if (!intakeMode && System.currentTimeMillis() >= timeSince6 + 1000) {
                                        intake.setPower(-1);
                                        intakeMode = true;
                                        timeSince6 = System.currentTimeMillis();
                                    }else if (intakeMode && System.currentTimeMillis() >= timeSince6 + 1000) {
                                        intake.setPower(0);
                                        timeSince6 = System.currentTimeMillis();
                                        intakeMode = false;
                                    }
                                }
                                Drive();
                                //intake.setPower(0);
                                liftPush.setPosition(0.7);
                            }
                            break;
                        } else if (gamepad1.x) {
                            telemetry.addData("Tip", "");
                            telemetry.update();
                            break;
                        }
                        telemetry.update();
                    }
                }
                // Get the current time in milliseconds. The value returned represents
                // the number of milliseconds since midnight, January 1, 1970 UTC.

                // Get the current time in milliseconds. The value returned represents
                // the number of milliseconds since midnight, January 1, 1970 UTC.
                // Get the current time in milliseconds. The value returned represents
                // the number of milliseconds since midnight, January 1, 1970 UTC.
                if (gamepad1.a && System.currentTimeMillis() >= 300 + timeSince3) {
                    if (servoPos1 < 0) {servoPos1 = 0.0;}
                    if (servoPos1 > 1) {servoPos1 = 1.0;}
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    timeSince3 = System.currentTimeMillis();
                    launchTurn.setPosition(servoPos1 + 0.1);// TODO: CHANGE ME
                    servoPos1 +=0.1;
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    timeSince2 = System.currentTimeMillis();
                } else if (gamepad1.y && System.currentTimeMillis() >= 300 + timeSince3) {
                    if (servoPos1 < 0) {servoPos1 = 0.0;}
                    if (servoPos1 > 1) {servoPos1 = 1.0;}
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    timeSince3 = System.currentTimeMillis();
                    launchTurn.setPosition(servoPos1 - 0.1); // TODO: CHANGE ME
                    servoPos1 -=0.1;
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    timeSince2 = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() >= 150 + timeSince2) {
                    telemetry.update();
                }
                while (gamepad1.dpad_up) {
                    if (!sensorLiftLimit.isPressed()) {
                        launchPush.setPosition(0.5);
                        ((DcMotorEx) liftUp).setVelocity(-250);
                    } else {
                        ((DcMotorEx) liftUp).setVelocity(0);
                    }
                }
                ((DcMotorEx) liftUp).setVelocity(0);
                while (gamepad1.dpad_down && !liftTouch.isPressed()) {
                    launchPush.setPosition(0.5);
                    ((DcMotorEx) liftUp).setVelocity(250);
                }
                ((DcMotorEx) liftUp).setVelocity(0);
                if (gamepad1.dpad_left) {
                    if (!intakeMode && System.currentTimeMillis() >= timeSince6 + 1000) {
                        intake.setPower(-1);
                        intakeMode = true;
                        timeSince6 = System.currentTimeMillis();
                    }else if (intakeMode && System.currentTimeMillis() >= timeSince6 + 1000) {
                        intake.setPower(0);
                        timeSince6 = System.currentTimeMillis();
                        intakeMode = false;
                    }
                }
                //intake.setPower(0);
                while (gamepad1.dpad_right) {
                    launchPush.setPosition(0.4);

                    liftPush.setPosition(0);

                }
                liftPush.setPosition(0.7);
//                if (!gamepad2.a) {
//                    sleep(500);
//                    liftPush.setPosition(0.7);
//                    liftState = "Unconfigured";
//                }
                //liftPush.setPosition(0.7);
                //boolean pressed = liftTouch.isPressed();
                if (liftTouch.isPressed()) {
                    telemetry.addData("liftTouch Status", "Pressed");
                } else {
                    telemetry.addData("liftTouch Status", "Not pressed");
                }
                if (sensorLiftLimit.isPressed()) {
                    telemetry.addData("SLL Status", "At limit");
                } else {
                    telemetry.addData("SLL Status", "Not at limit");
                }
                telemetry.addData("intake on", intake.getPower() < 0);
                telemetry.addData("Launcher angle", launchTurn.getPosition());
                //if (gamepad2.a) { driveGamepad = 2;}
                /*if (gamepad2.a) {
                    getStates();
                    switch (liftState) {
                        case "Unconfigured":

                            if (Boolean.TRUE.equals(states.get("atBottom")) && (Boolean.TRUE.equals(states.get("greenBallDetectBottom")) || Boolean.TRUE.equals(states.get("pplBallDetectBottom")))) {
                                // If lift is at bottom and ball is detected AT BOTTOM OF lift
//                            if (Boolean.TRUE.equals(states.get("greenBallDetectTop")) || Boolean.TRUE.equals(states.get("pplBallDetectTop"))) {
//                                // If a ball is detected ON LIFT (lift is at top)
//                                if (!runOnce){
//                                    launchPush.setPosition(0.4);
//                                    liftPush.setPosition(0);
//                                    timeSince4 = System.currentTimeMillis();
//                                    runOnce =true;
//                                }
//
//                                if (System.currentTimeMillis() >= timeSince4 + 1000) {
//
//                                }
//
//                            }
                                // If lift is at bottom then balls cannot be at top, so above is impossible
                                liftState = "D-B";// Down, ball

                            } else if (Boolean.TRUE.equals(states.get("atBottom")) && !(Boolean.TRUE.equals(states.get("greenBallDetectBottom")) || Boolean.TRUE.equals(states.get("pplBallDetectBottom")))){
                                liftState = "D-N"; // Down, no ball

                            } else if (Boolean.TRUE.equals(states.get("atTop")) && (Boolean.TRUE.equals(states.get("greenBallDetectTop")) || Boolean.TRUE.equals(states.get("pplBallDetectTop")))){
                                liftState = "U-B";

                            } else if (Boolean.TRUE.equals(states.get("atTop")) && !(Boolean.TRUE.equals(states.get("greenBallDetectTop")) || Boolean.TRUE.equals(states.get("pplBallDetectTop")))){
                                liftState = "U-N";

                            }
                            break;

                        case "D-B": //Down, ball
                            telemetry.addData("D", "getStates finished");
                            while (!sensorLiftLimit.isPressed()) {//Lift until at launch height

                                launchPush.setPosition(0.5);
                                ((DcMotorEx) liftUp).setVelocity(-250);
                            }
                            ((DcMotorEx) liftUp).setVelocity(0);
                            liftState = "U-B"; //Now the lift is up and the ball is still in
                            break;
                        case "U-B":
                            telemetry.addData("Case U-B", "getStates finished");
                            //if (!runOnce){
                                launchPush.setPosition(0.4); // open tray
                            waitTime1 = 1000;
                            timeZero = System.currentTimeMillis();
                            returnState1 = "lifted1";
                            liftState = "Wait";
                            //    timeSince4 = System.currentTimeMillis(); // start waiting
                            //    runOnce =true;
                            //}
                            //if (System.currentTimeMillis() >= timeSince4 + 1000) { // if 1s passed
                            //    telemetry.addData("Thing", "1s passed");
                            //    telemetry.update();
                            //    liftState = "lifted1";
                            //}
                            break;
                        case "lifted1":
                            telemetry.addData("lifted1", "getStates finished");
                            //if (Boolean.TRUE.equals(states.get("greenBallDetectTop")) || Boolean.TRUE.equals(states.get("pplBallDetectTop"))) { //Ball failed to be pushed in.
                                liftPush.setPosition(0); //push
                                liftState = "Wait";
                                waitTime1 = 1000;
                                timeZero = System.currentTimeMillis();
                                returnState1 = "liftretract";
                                runOnce = false;
                            //}
//                            } else {
//                                liftPush.setPosition(0.7); //retract
//                                liftState = "End";
//                            }
                            break;
                        case "liftretract":
                            telemetry.addData("liftretract", "getStates finished");
                            liftPush.setPosition(0.7); //retracted
                            liftState = "goDown";//need to go down here
                            launcherState = "Off-In";
                            break;
                        case "goDown":
                            while (!liftTouch.isPressed()) {
                                launchPush.setPosition(0.5);
                                ((DcMotorEx) liftUp).setVelocity(250);
                            }
                            ((DcMotorEx) liftUp).setVelocity(0);
                            break;
                        case "End":
                            telemetry.addData("End", "getStates finished");
                            break;
                        case "Wait":
                            //if (!runOnce2) {

                            //    runOnce2 = true;
                            //}
                            telemetry.addData("Waiting", "getStates finished");
                            if (System.currentTimeMillis() >= timeZero + waitTime1) {
                                liftState = returnState1;
                                break;
                            }
                    }
                    telemetry.update();


                switch (launcherState) {
                    case "Off-In":
                        ((DcMotorEx) rightLaunch).setVelocity(1500);
                        ((DcMotorEx) leftLaunch).setVelocity(-1500);
                        sleep(2000);
                        //launch motors on
                        launchPush.setPosition(1);
                        sleep(700);
                        launchPush.setPosition(0.5);
                        ((DcMotorEx) rightLaunch).setVelocity(0);
                        ((DcMotorEx) leftLaunch).setVelocity(0); // turn off motors
                        launcherState = "";
                        break;
                    case "On-In":
                        launchPush.setPosition(1);
                        sleep(700);
                        launchPush.setPosition(0.5);
                        launcherState = "";
                        ((DcMotorEx) rightLaunch).setVelocity(0);
                        ((DcMotorEx) leftLaunch).setVelocity(0); // turn off motors
                        break;
                    default:
                        telemetry.update();
                        break;
                }
                }*/
            }
        }
    }

    private void getStates() {
        states.put("atBottom", liftTouch.isPressed());
        states.put("atTop", sensorLiftLimit.isPressed());
        states.put("greenBallDetectBottom", sensorIntake.green() >= 115);
        states.put("pplBallDetectBottom", sensorIntake.blue() >= 115);
        states.put("pplBallDetectTop", sensorLift.blue() >= 84);
        states.put("greenBallDetectTop", sensorLift.green() >=84);
    }

    private void Drive() {
        float forwardBack;
        float strafe;
        float turn;
        float leftFrontPower;
        float rightFrontPower;
        float leftBackPower;
        float rightBackPower;

        // Determining movement based on gamepad inputs
        if (driveGamepad == 1) {forwardBack = -gamepad1.left_stick_y;} else {forwardBack = -gamepad2.left_stick_y;}
        if (driveGamepad == 1) {strafe = gamepad1.left_stick_x;} else {strafe = gamepad2.left_stick_x;}
        if (driveGamepad == 1) {turn = gamepad1.right_stick_x;} else { turn = gamepad2.right_stick_x;}
        leftFrontPower = forwardBack + strafe + turn;
        rightFrontPower = (forwardBack - strafe) - turn;
        leftBackPower = (forwardBack - strafe) + turn;
        rightBackPower = (forwardBack + strafe) - turn;
        // Setting Motor Power
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}