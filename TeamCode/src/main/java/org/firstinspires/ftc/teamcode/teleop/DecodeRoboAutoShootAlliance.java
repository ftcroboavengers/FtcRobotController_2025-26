package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Decode Robo AutoShoot Alliance", group = "RoboAvengers")
public class DecodeRoboAutoShootAlliance extends LinearOpMode {

    // ---------------- Drive ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ---------------- Intake / Launch ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- PIDF + RPM ----------------
    private static final double TPR = 28.0;      // ticks per revolution
    private static double rpmToTPS(double rpm) { return rpm * TPR / 60.0; }

    private double launcherTargetTPS = rpmToTPS(3000);  // default target
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;

    // Tuned F for your robot
    private static final double FIXED_F = 18.0;

    // Ready check
    private static final double VEL_TOL = 200.0;
    private static final int READY_CYCLES = 2;
    private int leftReadyCount = 0, rightReadyCount = 0;

    // Timings
    private static final double FEED_TIME_SEC    = 1.0;
    private static final double STOP_DELAY_SEC   = 0.25;
    private static final double REVERSE_TIME_SEC = 1.0;
    private static final double FEED_POWER       = 1.0;

    // ---------------- Manual FSM ----------------
    private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING, REVERSE }
    private LaunchState leftState  = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;

    private ElapsedTime leftTimer  = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();

    // ---------------- Auto Shoot FSM ----------------
    private enum AutoShootState { IDLE, FIND_TAG, AIMING, SPINUP, FEEDING, STOPPING }
    private AutoShootState autoState = AutoShootState.IDLE;

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime feedTimer = new ElapsedTime();

    // ---------------- Alliance + AprilTag IDs ----------------
    private boolean allianceSelected = false;
    private boolean isRedAlliance    = false;   // value only matters if allianceSelected = true

    private static final int RED_CENTER_TAG  = 24;
    private static final int BLUE_CENTER_TAG = 20;

    private static final double AIM_TOL_DEG = 3.5;
    private static final int AIM_STABLE_LOOPS = 2;
    private int aimStableCount = 0;

    private double lastTagDistanceM = -1.0;
    private double lastTagBearingDeg = 0.0;

    // ---------------- Strong Smoothing for AutoAim ----------------
    private double smoothedDistance = -1;
    private double smoothedBearing = 0;

    private double lastRawDistance = -1;
    private static final double MAX_DISTANCE_JUMP = 0.20;   // 20cm (8 inches) spike rejection

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ---------------- Unjam ----------------
    private boolean unjamming = false;
    private ElapsedTime unjamTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Hardware ----------------
        leftFront  = getMotor("front_left_drive");
        rightFront = getMotor("front_right_drive");
        leftBack   = getMotor("back_left_drive");
        rightBack  = getMotor("back_right_drive");

        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // Drive directions
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Intake direction
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Launchers
        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        // AprilTags
        initAprilTags();

        // ---------------- Alliance Select BEFORE start ----------------
        telemetry.addLine("INIT: Select Alliance for AutoShoot");
        telemetry.addLine("Gamepad1 B = RED (tag 24)");
        telemetry.addLine("Gamepad1 A = BLUE (tag 20)");
        telemetry.addLine("If no selection -> AutoShoot DISABLED");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            // NOTE: This is currently A = RED, B = BLUE in code
            if (gamepad1.a) {
                isRedAlliance = true;
                allianceSelected = true;
            }
            if (gamepad1.b) {
                isRedAlliance = false;
                allianceSelected = true;
            }

            telemetry.addData("Alliance",
                    allianceSelected
                            ? (isRedAlliance ? "RED (ID 24)" : "BLUE (ID 20)")
                            : "NOT SELECTED");
            telemetry.addData("AutoShoot", allianceSelected ? "ENABLED" : "DISABLED");
            telemetry.update();
        }

        if (isStopRequested()) return;

        telemetry.clearAll();
        if (!allianceSelected) {
            telemetry.addLine("WARNING: NO ALLIANCE SELECTED");
            telemetry.addLine("AutoShoot WILL NOT activate this match.");
        } else {
            telemetry.addData("Alliance Locked", isRedAlliance ? "RED (24)" : "BLUE (20)");
            telemetry.addLine("AutoShoot ENABLED");
        }
        telemetry.update();

        if (isStopRequested()) return;

        // Set PIDF after start
        setPIDF(leftLauncher, FIXED_F);
        setPIDF(rightLauncher, FIXED_F);

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // --------------- UNJAM (X) ---------------
            boolean unjamPressed = gamepad2.x;
            if (unjamPressed && autoState == AutoShootState.IDLE && !unjamming) {
                startUnjam();
            }
            if (unjamming) {
                updateUnjam();
            }

            // --------------- DRIVE ---------------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05)  y = 0;
            if (Math.abs(x) < 0.05)  x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double driveScale = gamepad1.right_bumper ? 1.0 :
                    gamepad1.left_bumper  ? 0.4 : 0.7;

            if (autoState == AutoShootState.AIMING) {
                AprilTagDetection tag = getBestAllianceTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagDistanceM  = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                }
                double turnPower = aimTurnPower(lastTagBearingDeg);
                drive(0, 0, turnPower, driveScale);
            }
            else if (autoState == AutoShootState.IDLE) {
                drive(y, x, rx, driveScale);
            }
            else {
                drive(0, 0, 0, 1.0);
            }

            // --------------- INTAKE (gamepad1 triggers) ---------------
            if (!unjamming
                    && autoState == AutoShootState.IDLE
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE) {

                double in  = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;
                double p   = (in - out) * 1.0;

                if (Math.abs(in) < 0.02 && Math.abs(out) < 0.02) {
                    p = 0;
                }

                if (intake != null) intake.setPower(p);
            }

            // --------------- AUTO SHOOT START (D-PAD UP) ---------------
            if (gamepad2.dpad_up
                    && allianceSelected                 // SAFETY: requires A/B at init
                    && autoState == AutoShootState.IDLE
                    && !unjamming) {

                // Make sure everything is in a known safe state
                stopLaunch();
                if (intake != null) intake.setPower(0);
                leftState  = LaunchState.IDLE;
                rightState = LaunchState.IDLE;

                startAutoShoot();
            }

            // Cancel auto shoot with D-pad DOWN
            if (gamepad2.dpad_down) {
                cancelAutoShoot();
            }

            // --------------- MANUAL FSM LAUNCHING (bumpers) ---------------
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // --------------- MANUAL PRESETS (A/B/Y) ---------------
            if (autoState == AutoShootState.IDLE && !unjamming) {

                // A = SHORT (2900 rpm)
                if (gamepad2.a) {
                    launcherTargetTPS = rpmToTPS(2900);
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);
                    telemetry.addLine("Preset: SHORT (2900 rpm, F=18)");
                }

                // B = MID (3000 rpm)
                if (gamepad2.b) {
                    launcherTargetTPS = rpmToTPS(3000);
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);
                    telemetry.addLine("Preset: MID (3000 rpm, F=18)");
                }

                // Y = LONG (3500 rpm)
                if (gamepad2.y) {
                    launcherTargetTPS = rpmToTPS(3500);
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);
                    telemetry.addLine("Preset: LONG (3500 rpm, F=18)");
                }
            }

            // --------------- MANUAL HOLD-SPIN (TRIGGERS) ---------------
            if (autoState == AutoShootState.IDLE && !unjamming) {

                // NEW: Update RPM automatically based on distance
                updateManualDistanceRPM();

                // Left launcher
                if (leftLauncher != null && leftState == LaunchState.IDLE) {
                    if (gamepad2.left_trigger > 0.2) {
                        leftLauncher.setVelocity(voltageComp(launcherTargetTPS));
                    } else if (!gamepad2.left_bumper) {
                        leftLauncher.setPower(0);
                    }
                }

                // Right launcher
                if (rightLauncher != null && rightState == LaunchState.IDLE) {
                    if (gamepad2.right_trigger > 0.2) {
                        rightLauncher.setVelocity(voltageComp(launcherTargetTPS));
                    } else if (!gamepad2.right_bumper) {
                        rightLauncher.setPower(0);
                    }
                }
            }

            // --------------- MANUAL FEED (gamepad2 right stick) ---------------
            if (autoState == AutoShootState.IDLE
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE
                    && !unjamming) {

                if (gamepad2.right_stick_button) {
                    if (intake != null) intake.setPower(FEED_POWER);
                } else if (Math.abs(gamepad1.left_trigger) < 0.01
                        && Math.abs(gamepad1.right_trigger) < 0.01) {
                    if (intake != null) intake.setPower(0);
                }
            }

            // --------------- PANIC STOP (BACK) ---------------
            if (gamepad2.back) {
                stopLaunch();
                if (intake != null) intake.setPower(0);
                leftState  = LaunchState.IDLE;
                rightState = LaunchState.IDLE;
                unjamming = false;
                autoState = AutoShootState.IDLE;
            }

            // --------------- AUTO SHOOT FSM ---------------
            updateAutoShoot();

            // --------------- TELEMETRY ---------------
            telemetry.addData("Alliance",
                    allianceSelected
                            ? (isRedAlliance ? "RED (24)" : "BLUE (20)")
                            : "NOT SELECTED");
            telemetry.addData("AutoShoot", allianceSelected ? "ENABLED" : "DISABLED");
            telemetry.addData("AutoState", autoState);
            telemetry.addData("LeftState", leftState);
            telemetry.addData("RightState", rightState);
            telemetry.addData("TargetRPM", launcherTargetTPS * 60.0 / TPR);
            telemetry.addData("LeftVel",  leftLauncher  != null ? leftLauncher.getVelocity()  : 0);
            telemetry.addData("RightVel", rightLauncher != null ? rightLauncher.getVelocity() : 0);
            telemetry.addData("Bearing",  lastTagBearingDeg);
            telemetry.addData("Distance", lastTagDistanceM);
            telemetry.addData("SmoothedBearing", smoothedBearing);
            telemetry.addData("SmoothedDistance", smoothedDistance);
            telemetry.addData("AimStableCount", aimStableCount);
            telemetry.addData("Unjamming", unjamming);
            if (!allianceSelected) {
                telemetry.addLine("AUTO SHOOT DISABLED – NO ALLIANCE SELECTED");
            }
            telemetry.update();
        }

        stopAll();
    }

    // ============================================================
    // AUTO SHOOT LOGIC
    // ============================================================

    private void startAutoShoot() {
        autoState = AutoShootState.FIND_TAG;
        autoTimer.reset();
        aimStableCount = 0;
        smoothedDistance = -1;
        smoothedBearing = 0;
        lastRawDistance = -1;
    }

    private void cancelAutoShoot() {
        autoState = AutoShootState.IDLE;
        if (intake != null) intake.setPower(0);
        stopLaunch();
        aimStableCount = 0;
    }

    private void updateAutoShoot() {
        switch (autoState) {

            case IDLE:
                return;

            case FIND_TAG: {
                AprilTagDetection tag = getBestAllianceTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagDistanceM  = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                    autoTimer.reset();              // reset aim timer when we first see the tag
                    aimStableCount = 0;
                    autoState = AutoShootState.AIMING;
                } else if (autoTimer.seconds() > 2.0) {
                    cancelAutoShoot();
                }
                return;
            }

            case AIMING: {

                AprilTagDetection tag2 = getBestAllianceTag();
                if (tag2 != null && tag2.ftcPose != null) {

                    double rawDistInches = tag2.ftcPose.range;

                    double rawDist = rawDistInches * 0.0254;   // convert inches → meters

                    double rawBear = tag2.ftcPose.bearing;

                    // ---------------- Spike Rejection (distance) ----------------
                    if (lastRawDistance > 0 && Math.abs(rawDist - lastRawDistance) > MAX_DISTANCE_JUMP) {
                        // too large of a jump → ignore this frame
                        break;
                    }
                    lastRawDistance = rawDist;

                    // ---------------- Strong Smoothing ----------------
                    // Distance
                    if (smoothedDistance < 0) {
                        smoothedDistance = rawDist;     // initialize on first good frame
                    } else {
                        smoothedDistance = 0.7 * smoothedDistance + 0.3 * rawDist;
                    }

                    // Bearing
                    smoothedBearing = 0.7 * smoothedBearing + 0.3 * rawBear;

                    // Assign to your normal variables for telemetry + turn logic
                    lastTagDistanceM  = smoothedDistance;
                    lastTagBearingDeg = smoothedBearing;
                } else {
                    // If lost → go back and search again
                    autoState = AutoShootState.FIND_TAG;
                    autoTimer.reset();
                    aimStableCount = 0;
                    return;
                }

                double elapsedAim = autoTimer.seconds();

                // ---------------- Smart "near enough" fallback ----------------
                // If we've been aiming for a bit and we're close to centered,
                // force the alignment so we don't get stuck forever.
                if (Math.abs(smoothedBearing) < 7.0 && elapsedAim > 1.2) {
                    aimStableCount = AIM_STABLE_LOOPS;
                }

                // ---------------- Normal Stability Check ----------------
                if (Math.abs(smoothedBearing) <= 2.0) {   // widened tolerance for noise
                    aimStableCount++;
                } else {
                    aimStableCount = 0;
                }

                // ---------------- Final timeout safety ----------------
                // If we've been trying to aim for > 2.0 seconds, just shoot.
                if (elapsedAim > 2.0) {
                    aimStableCount = AIM_STABLE_LOOPS;
                }

                // ---------------- When stable, choose RPM and shoot ----------------
                if (aimStableCount >= 1) {      // only 1 stable frame needed now
                    double d = smoothedDistance;

                    // --- REAL TEAM CALIBRATED BUCKETS ---
                    // short: under 3 ft (0.9144 m)
                    // mid:   3 ft to 9 ft (0.9144 to 2.7432 m)
                    // long:  9+ ft (2.7432+ m)

                    if (d < 0.9144) {
                        launcherTargetTPS = rpmToTPS(2900);    // SHORT
                    }
                    else if (d < 2.7432) {
                        launcherTargetTPS = rpmToTPS(3000);    // MID
                    }
                    else {
                        launcherTargetTPS = rpmToTPS(3500);    // LONG
                    }

                    // Apply motor PIDF gain
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);

                    // Spin-up both launchers
                    startLaunch();

                    autoTimer.reset();
                    autoState = AutoShootState.SPINUP;
                }

                return;
            }

            case SPINUP: {
                boolean ready = leftReady() && rightReady();
                if (ready && autoTimer.seconds() > 0.3) {
                    if (intake != null) intake.setPower(FEED_POWER);
                    feedTimer.reset();
                    autoState = AutoShootState.FEEDING;
                } else if (autoTimer.seconds() > 3.0) {
                    cancelAutoShoot();
                }
                return;
            }

            case FEEDING:
                if (feedTimer.seconds() > FEED_TIME_SEC) {
                    if (intake != null) intake.setPower(0);
                    autoTimer.reset();
                    autoState = AutoShootState.STOPPING;
                }
                return;

            case STOPPING:
                if (autoTimer.seconds() > STOP_DELAY_SEC) {
                    stopLaunch();
                    autoState = AutoShootState.IDLE;
                }
        }
    }

    // ============================================================
    // MANUAL FSM LAUNCH LOGIC
    // ============================================================

    private void launchLeft(boolean shoot, boolean unjamPressed) {
        if (leftLauncher == null || intake == null) return;
        if (autoState != AutoShootState.IDLE) return;
        if (unjamming) return;   // do not fight unjam

        switch (leftState) {

            case IDLE:
                if (shoot) {
                    startLeft();
                    leftTimer.reset();
                    leftState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (leftReady() && leftTimer.seconds() > 0.3) {
                    intake.setPower(FEED_POWER);
                    leftTimer.reset();
                    leftState = LaunchState.LAUNCHING;
                } else if (!leftReady()) {
                    leftTimer.reset();
                }
                break;

            case LAUNCHING:
                if (unjamPressed) { startUnjam(); break; }
                if (leftTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    leftTimer.reset();
                    leftState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (leftTimer.seconds() > STOP_DELAY_SEC) {
                    stopLeft();
                    leftState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                // handled by unjam timer
                break;
        }
    }

    private void launchRight(boolean shoot, boolean unjamPressed) {
        if (rightLauncher == null || intake == null) return;
        if (autoState != AutoShootState.IDLE) return;
        if (unjamming) return;   // do not fight unjam

        switch (rightState) {

            case IDLE:
                if (shoot) {
                    startRight();
                    rightTimer.reset();
                    rightState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (rightReady() && rightTimer.seconds() > 0.3) {
                    intake.setPower(FEED_POWER);
                    rightTimer.reset();
                    rightState = LaunchState.LAUNCHING;
                } else if (!rightReady()) {
                    rightTimer.reset();
                }
                break;

            case LAUNCHING:
                if (unjamPressed) { startUnjam(); break; }
                if (rightTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    rightTimer.reset();
                    rightState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (rightTimer.seconds() > STOP_DELAY_SEC) {
                    stopRight();
                    rightState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                // handled by unjam timer
                break;
        }
    }

    // ============================================================
    // LAUNCHER HELPERS
    // ============================================================

    private void initLauncher(DcMotorEx m, boolean reverse) {
        if (m == null) return;
        m.setZeroPowerBehavior(BRAKE);
        m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPIDF(DcMotorEx m, double f) {
        if (m != null) {
            m.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, f);
        }
    }

    private double voltageComp(double targetTPS) {
        double nominal = 13.0;
        double v = 12.0;
        try {
            v = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception ignored) {}
        if (v <= 0) v = 12.0;
        return targetTPS * (nominal / v);
    }

    private boolean leftReady() {
        double v = leftLauncher.getVelocity();
        boolean good = Math.abs(v - launcherTargetTPS) <= VEL_TOL;
        leftReadyCount = good ? Math.min(READY_CYCLES, leftReadyCount + 1) : 0;
        return leftReadyCount >= READY_CYCLES;
    }

    private boolean rightReady() {
        double v = rightLauncher.getVelocity();
        boolean good = Math.abs(v - launcherTargetTPS) <= VEL_TOL;
        rightReadyCount = good ? Math.min(READY_CYCLES, rightReadyCount + 1) : 0;
        return rightReadyCount >= READY_CYCLES;
    }

    private void startLaunch() {
        if (leftLauncher != null)
            leftLauncher.setVelocity(voltageComp(launcherTargetTPS));
        if (rightLauncher != null)
            rightLauncher.setVelocity(voltageComp(launcherTargetTPS));
        leftReadyCount = 0;
        rightReadyCount = 0;
    }

    private void stopLaunch() {
        stopLeft();
        stopRight();
    }

    private void startLeft() {
        if (leftLauncher != null) {
            leftLauncher.setVelocity(voltageComp(launcherTargetTPS));
        }
    }

    private void startRight() {
        if (rightLauncher != null) {
            rightLauncher.setVelocity(voltageComp(launcherTargetTPS));
        }
    }

    private void stopLeft() {
        if (leftLauncher != null) {
            leftLauncher.setPower(0);
        }
    }

    private void stopRight() {
        if (rightLauncher != null) {
            rightLauncher.setPower(0);
        }
    }

    // ============================================================
    // MANUAL DISTANCE-BASED RPM LOGIC
    // ============================================================
    private void updateManualDistanceRPM() {
        // Only update when we actually see the tag
        AprilTagDetection tag = getBestAllianceTag();
        if (tag == null || tag.ftcPose == null) return;

        // Convert inches → meters
        double d = tag.ftcPose.range * 0.0254;

        // Same buckets you tested
        if (d < 0.9144) {
            launcherTargetTPS = rpmToTPS(2900);   // SHORT
        }
        else if (d < 2.7432) {
            launcherTargetTPS = rpmToTPS(3000);   // MID
        }
        else {
            launcherTargetTPS = rpmToTPS(3500);   // LONG
        }

        // Refresh PIDF
        setPIDF(leftLauncher, FIXED_F);
        setPIDF(rightLauncher, FIXED_F);
    }


    // ============================================================
    // UNJAM (X)
    // ============================================================

    private void startUnjam() {
        unjamming = true;
        unjamTimer.reset();

        // Reverse ONLY the launchers, keep intake off
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
        if (intake != null)        intake.setPower(0);

        leftState  = LaunchState.REVERSE;
        rightState = LaunchState.REVERSE;
        leftTimer.reset();
        rightTimer.reset();
    }

    private void updateUnjam() {
        if (unjamTimer.seconds() > REVERSE_TIME_SEC) {
            unjamming = false;
            stopLaunch();
            if (intake != null) intake.setPower(0);
            leftState  = LaunchState.IDLE;
            rightState = LaunchState.IDLE;
        }
    }

    // ============================================================
    // APRILTAG VISION
    // ============================================================

    private void initAprilTags() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .build();
    }

    /**
     * Returns the best tag for the CURRENT alliance:
     *   - Red  -> ID 24
     *   - Blue -> ID 20
     * If no alliance selected, returns null (safety).
     */
    private AprilTagDetection getBestAllianceTag() {
        if (!allianceSelected) return null;    // SAFETY: no alliance, no tag
        if (tagProcessor == null) return null;

        int targetId = isRedAlliance ? RED_CENTER_TAG : BLUE_CENTER_TAG;

        // Snapshot list to avoid ConcurrentModificationException
        List<AprilTagDetection> detections =
                new ArrayList<>(tagProcessor.getDetections());

        for (AprilTagDetection tag : detections) {
            if (tag.ftcPose != null && tag.id == targetId) {
                return tag;
            }
        }
        return null;
    }

    private double aimTurnPower(double bearingDeg) {

        // Stronger proportional control
        double kP = 0.035;   // was 0.02 → turning was too weak

        // Compute raw turn power
        double turn = -kP * bearingDeg;

        // Minimum power so robot actually MOVES
        double minPower = 0.14;   // was 0.12 but too weak after smoothing

        // Maximum turn power
        double maxPower = 0.45;   // small boost from 0.40

        // Apply minimum power threshold
        if (Math.abs(turn) < minPower && Math.abs(bearingDeg) > 3.0) {
            // far away → use minimum
            turn = Math.signum(turn) * minPower;
        }

        // Scale down as we get close (< 10 deg)
        if (Math.abs(bearingDeg) < 10) {
            turn *= 0.6;  // smoother final approach
        }

        // Cap the power
        if (Math.abs(turn) > maxPower) {
            turn = Math.signum(turn) * maxPower;
        }

        return turn;
    }

    // ============================================================
    // DRIVE
    // ============================================================

    private void drive(double y, double x, double rx, double scale) {
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denom * scale;
        double bl = (y - x + rx) / denom * scale;
        double fr = (y - x - rx) / denom * scale;
        double br = (y + x - rx) / denom * scale;

        setPower(fl, fr, bl, br);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private void stopAll() {
        stopLaunch();
        if (intake != null) intake.setPower(0);
        setPower(0, 0, 0, 0);
    }

    // ============================================================
    // SAFE HARDWARE GETTERS
    // ============================================================

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}
