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

@TeleOp(name = "Decode RoboAvengers Alliance", group = "RoboAvengers")
public class DecodeRoboAutoShootAlliance2 extends LinearOpMode {

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

    // ---------------- AprilTag IDs ----------------
    private static final int RED_CENTER_TAG  = 24;
    private static final int BLUE_CENTER_TAG = 20;

    // Tag info for telemetry
    private double lastTagDistanceM = -1.0;
    private int    lastTagId        = -1;
    private double lastTagRecommendedRPM = 3000;

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

        telemetry.addLine("INIT: Tag-based RPM TeleOp (No AutoShoot)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Set PIDF after start
        setPIDF(leftLauncher, FIXED_F);
        setPIDF(rightLauncher, FIXED_F);

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // --------------- UNJAM (X) ---------------
            boolean unjamPressed = gamepad2.x;
            if (unjamPressed && !unjamming) {
                startUnjam();
            }
            if (unjamming) {
                updateUnjam();
            }

            // --------------- DRIVE (GAMEPAD 1) ---------------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05)  y = 0;
            if (Math.abs(x) < 0.05)  x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double driveScale = gamepad1.right_bumper ? 1.0 :
                    gamepad1.left_bumper  ? 0.4 : 0.7;

            drive(y, x, rx, driveScale);

            // --------------- INTAKE (GAMEPAD 1 TRIGGERS) ---------------
            if (!unjamming
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

            // --------------- MANUAL FSM LAUNCHING (bumpers) ---------------
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // --------------- MANUAL PRESETS (A/B/Y) ---------------
            if (!unjamming) {
                // A = SHORT (2900 rpm)
                if (gamepad2.a) {
                    launcherTargetTPS = rpmToTPS(2900);
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);
                    telemetry.addLine("Preset: SHORT (2900 rpm, F=18)");
                    lastTagRecommendedRPM = 2900;
                }

                // B = MID (3000 rpm)
                if (gamepad2.b) {
                    launcherTargetTPS = rpmToTPS(3000);
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);
                    telemetry.addLine("Preset: MID (3000 rpm, F=18)");
                    lastTagRecommendedRPM = 3000;
                }

                // Y = LONG (3800 rpm)
                if (gamepad2.y) {
                    launcherTargetTPS = rpmToTPS(3800);
                    setPIDF(leftLauncher, FIXED_F);
                    setPIDF(rightLauncher, FIXED_F);
                    telemetry.addLine("Preset: LONG (3800 rpm, F=18)");
                    lastTagRecommendedRPM = 3800;
                }
            }

            // --------------- MANUAL HOLD-SPIN (TRIGGERS, GAMEPAD 2) ---------------
            if (!unjamming) {

                // Update RPM automatically based on distance when we're using triggers
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

            // --------------- MANUAL FEED (gamepad2 right stick button) ---------------
            if (leftState == LaunchState.IDLE
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
            }

            // --------------- TELEMETRY ---------------
            AprilTagDetection tag = getBestTag();
            if (tag != null && tag.ftcPose != null) {
                double distM = tag.ftcPose.range * 0.0254;
                lastTagDistanceM = distM;
                lastTagId = tag.id;

                telemetry.addData("Tag Seen",
                        tag.id == BLUE_CENTER_TAG ? "BLUE 20" :
                                tag.id == RED_CENTER_TAG  ? "RED 24" : tag.id);
                telemetry.addData("Tag Distance (m)", "%.2f", distM);
            } else {
                telemetry.addLine("Tag Seen: NONE");
            }

            telemetry.addData("RecommendedRPM (Tag)", "%.0f", lastTagRecommendedRPM);
            telemetry.addData("TargetRPM (Current)", "%.0f", launcherTargetTPS * 60.0 / TPR);
            telemetry.addData("LeftVel",  leftLauncher  != null ? leftLauncher.getVelocity()  : 0);
            telemetry.addData("RightVel", rightLauncher != null ? rightLauncher.getVelocity() : 0);
            telemetry.addData("LeftState", leftState);
            telemetry.addData("RightState", rightState);
            telemetry.addData("Unjamming", unjamming);
            telemetry.update();
        }

        stopAll();
    }

    // ============================================================
    // MANUAL FSM LAUNCH LOGIC
    // ============================================================

    private void launchLeft(boolean shoot, boolean unjamPressed) {
        if (leftLauncher == null || intake == null) return;
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
    // MANUAL DISTANCE-BASED RPM LOGIC (ANY RED/BLUE TAG)
    // ============================================================
    private void updateManualDistanceRPM() {
        // Only update when we actually see a center tag (red 24 or blue 20)
        AprilTagDetection tag = getBestTag();
        if (tag == null || tag.ftcPose == null) return;

        // Convert inches → meters
        double d = tag.ftcPose.range * 0.0254;
        lastTagDistanceM = d;
        lastTagId = tag.id;

        // Same buckets you tested:
        // short: under 3 ft (0.9144 m)
        // mid:   3 ft to 8.2021 ft (0.9144 to 2.500 m)
        // long:  8.2021+ ft (2.500+ m)
        double rpm;
        if (d < 0.9144) {
            rpm = 2900;   // SHORT
        }
        else if (d < 2.500) {
            rpm = 3000;   // MID
        }
        else {
            rpm = 3800;   // LONG
        }

        lastTagRecommendedRPM = rpm;
        launcherTargetTPS = rpmToTPS(rpm);

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
     * Returns the best center tag (red 24 or blue 20) with smallest |bearing|.
     * No alliance selection needed.
     */
    private AprilTagDetection getBestTag() {
        if (tagProcessor == null) return null;

        List<AprilTagDetection> detections =
                new ArrayList<>(tagProcessor.getDetections());

        AprilTagDetection best = null;
        double bestAbsBearing = Double.MAX_VALUE;

        for (AprilTagDetection tag : detections) {
            if (tag.ftcPose == null) continue;
            if (tag.id != RED_CENTER_TAG && tag.id != BLUE_CENTER_TAG) continue;

            double absBearing = Math.abs(tag.ftcPose.bearing);
            if (absBearing < bestAbsBearing) {
                bestAbsBearing = absBearing;
                best = tag;
            }
        }
        return best;
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
