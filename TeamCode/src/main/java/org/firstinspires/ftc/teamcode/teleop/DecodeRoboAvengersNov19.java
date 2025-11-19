package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Decode RoboAvengers Gameday", group = "RoboAvengers")
public class DecodeRoboAvengersNov19 extends LinearOpMode {

    // ---------------- Drive / Pinpoint ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private boolean fieldCentric = false;
    private double headingOffsetRad = 0.0;

    // ---------------- Intake / Launch ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- Launcher RPM / PIDF ----------------
    private static final double TPR = 28.0;
    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TPR / 60.0;
    }

    private static final double LAUNCH_CLOSE_RPM = 4000.0;
    private static final double LAUNCH_FAR_RPM   = 6000.0;

    // start in close shot mode
    private double launcherTargetTPS = rpmToTicksPerSec(LAUNCH_CLOSE_RPM);
    private boolean isFarShot = false;

    // Velocity tolerance / ready logic
    private static final int VEL_TOL = 250;
    private static final int READY_CYCLES = 2;
    private int leftReadyCount  = 0;
    private int rightReadyCount = 0;

    // PIDF gains (tune F per motor as needed)
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;
    private static double LEFT_F_GAIN  = 12.0;
    private static double RIGHT_F_GAIN = 12.0;

    // ---------------- Timings ----------------
    private static final double FEED_TIME_SEC    = 1.0;
    private static final double STOP_DELAY_SEC   = 0.25;
    private static final double REVERSE_TIME_SEC = 1.0;
    private static final double FEED_POWER       = 1.0;

    private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING, REVERSE }
    private LaunchState leftState  = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;

    private ElapsedTime leftTimer  = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();

    // toggle edge detection for A button
    private boolean prevShotToggleA = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Map hardware ----
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // ---- Drive setup ----
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Intake
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Launchers
        if (leftLauncher != null) {
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, LEFT_F_GAIN);
        }
        if (rightLauncher != null) {
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, RIGHT_F_GAIN);
        }

        // ---- Pinpoint ----
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
            zeroHeading();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found — field-centric disabled.");
            fieldCentric = false;
        }

        telemetry.addLine("TeleOp READY");
        telemetry.update();
        waitForStart();

        // Re-apply PIDF after start in case FTC SDK touches it
        if (leftLauncher != null) {
            leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, LEFT_F_GAIN);
        }
        if (rightLauncher != null) {
            rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, RIGHT_F_GAIN);
        }

        while (opModeIsActive()) {

            // -------- DRIVE --------
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Apply deadzone
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            if (gamepad1.x) fieldCentric = !fieldCentric;
            if (gamepad1.y) zeroHeading();

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);
            mecanumDrive(y, x, rx, driveScale);

            // -------- SHOT MODE TOGGLE (gamepad2 A) --------
            boolean aNow = gamepad2.a;
            if (aNow && !prevShotToggleA) {
                // toggle between close and far
                isFarShot = !isFarShot;
                double rpm = isFarShot ? LAUNCH_FAR_RPM : LAUNCH_CLOSE_RPM;
                launcherTargetTPS = rpmToTicksPerSec(rpm);
            }
            prevShotToggleA = aNow;

            // -------- INTAKE (only when not shooting) --------
            if (intake != null && leftState == LaunchState.IDLE && rightState == LaunchState.IDLE) {
                double in = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;

                double p = 0;
                double scale = 0.7;
                if (in > 0.01 || out > 0.01) {
                    p = (in - out) * scale;
                }
                intake.setPower(p);
            }

            // -------- CLEAR ALL (gamepad2 Y) --------
            if (gamepad2.y) {
                clearAll();
            }

            // -------- SHOOT / UNJAM --------
            boolean unjamPressed = gamepad2.x;
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // Manual unjam if idle on both sides and X pressed
            if (gamepad2.x
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE) {
                startReverseUnjam();
            }

            // -------- Telemetry --------
            if (pinpoint != null) {
                pinpoint.update();
                telemetry.addData("Heading(deg)", Math.toDegrees(getYawRad()));
                telemetry.addData("FieldCentric", fieldCentric);
            }

            telemetry.addData("Shot Mode", isFarShot ? "FAR" : "CLOSE");
            telemetry.addData("Target (tps)", launcherTargetTPS);
            telemetry.addData("Left Vel (tps)", leftLauncher != null ? leftLauncher.getVelocity() : 0.0);
            telemetry.addData("Right Vel (tps)", rightLauncher != null ? rightLauncher.getVelocity() : 0.0);
            telemetry.addData("Left State", leftState);
            telemetry.addData("Right State", rightState);
            telemetry.addData("Intake Power", intake != null ? intake.getPower() : 0.0);
            telemetry.update();
        }

        stopAll();
    }

    // ---------------- Helper Methods ----------------

    private void clearAll() {
        stopAll();
        leftState = LaunchState.IDLE;
        rightState = LaunchState.IDLE;
        leftTimer.reset();
        rightTimer.reset();
        leftReadyCount = 0;
        rightReadyCount = 0;
        telemetry.addLine("CLEAR ALL triggered — reset complete.");
        telemetry.update();
    }

    private void mecanumDrive(double y, double x, double rx, double scale) {
        if (leftFront == null) return;
        double rotX = x;
        double rotY = y;
        if (fieldCentric && pinpoint != null) {
            double yaw = getYawRad();
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);
            rotX = x * cosA - y * sinA;
            rotY = x * sinA + y * cosA;
        }
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denom * scale;
        double bl = (rotY - rotX + rx) / denom * scale;
        double fr = (rotY - rotX - rx) / denom * scale;
        double br = (rotY + rotX - rx) / denom * scale;
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
            if (m != null) {
                m.setZeroPowerBehavior(BRAKE);
            }
        }
    }

    private void zeroHeading() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            headingOffsetRad = 0.0;
        }
    }

    private double getYawRad() {
        if (pinpoint == null) return 0.0;
        return pinpoint.getHeading(AngleUnit.RADIANS) - headingOffsetRad;
    }

    private void stopAll() {
        stopLeftLauncher();
        stopRightLauncher();
        if (intake != null) intake.setPower(0);
        setPower(0, 0, 0, 0);
    }

    // ---------------- Launcher Control (RPM / PIDF) ----------------

    private void startLeftLauncher() {
        if (leftLauncher == null) return;
        double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setVelocity(adjusted);
    }

    private void startRightLauncher() {
        if (rightLauncher == null) return;
        double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setVelocity(adjusted);
    }

    private void stopLeftLauncher() {
        if (leftLauncher != null) {
            leftLauncher.setPower(0);
        }
    }

    private void stopRightLauncher() {
        if (rightLauncher != null) {
            rightLauncher.setPower(0);
        }
    }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0;
        double currentVoltage = 12.0;
        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception e) {
            // ignore, fallback to 12V
        }
        if (currentVoltage <= 0.0) {
            currentVoltage = 12.0;
        }
        return targetTicksPerSec * (nominalVoltage / currentVoltage);
    }

    private boolean leftReady() {
        if (leftLauncher == null) return false;
        double v = leftLauncher.getVelocity();
        boolean inTol = Math.abs(v - launcherTargetTPS) <= VEL_TOL;
        leftReadyCount = inTol ? Math.min(READY_CYCLES, leftReadyCount + 1) : 0;
        return leftReadyCount >= READY_CYCLES;
    }

    private boolean rightReady() {
        if (rightLauncher == null) return false;
        double v = rightLauncher.getVelocity();
        boolean inTol = Math.abs(v - launcherTargetTPS) <= VEL_TOL;
        rightReadyCount = inTol ? Math.min(READY_CYCLES, rightReadyCount + 1) : 0;
        return rightReadyCount >= READY_CYCLES;
    }

    private void startReverseUnjam() {
        if (intake != null) intake.setPower(0);
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
        leftState = LaunchState.REVERSE;
        rightState = LaunchState.REVERSE;
        leftTimer.reset();
        rightTimer.reset();
        leftReadyCount = 0;
        rightReadyCount = 0;
    }

    // Independent Launch Logic

    private void launchLeft(boolean shootRequested, boolean unjamRequested) {
        if (leftLauncher == null || intake == null) return;

        switch (leftState) {
            case IDLE:
                if (shootRequested) {
                    startLeftLauncher();
                    leftTimer.reset();
                    leftState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (leftReady()) {
                    if (leftTimer.seconds() > 0.3) {
                        intake.setPower(FEED_POWER);
                        leftTimer.reset();
                        leftState = LaunchState.LAUNCHING;
                    }
                } else {
                    leftTimer.reset();
                }
                break;

            case LAUNCHING:
                if (unjamRequested) {
                    startReverseUnjam();
                    break;
                }
                if (leftTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    leftTimer.reset();
                    leftState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (unjamRequested) {
                    startReverseUnjam();
                    break;
                }
                if (leftTimer.seconds() > STOP_DELAY_SEC) {
                    stopLeftLauncher();
                    leftState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                if (leftTimer.seconds() > REVERSE_TIME_SEC) {
                    stopLeftLauncher();
                    leftState = LaunchState.IDLE;
                }
                break;
        }
    }

    private void launchRight(boolean shootRequested, boolean unjamRequested) {
        if (rightLauncher == null || intake == null) return;

        switch (rightState) {
            case IDLE:
                if (shootRequested) {
                    startRightLauncher();
                    rightTimer.reset();
                    rightState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (rightReady()) {
                    if (rightTimer.seconds() > 0.3) {
                        intake.setPower(FEED_POWER);
                        rightTimer.reset();
                        rightState = LaunchState.LAUNCHING;
                    }
                } else {
                    rightTimer.reset();
                }
                break;

            case LAUNCHING:
                if (unjamRequested) {
                    startReverseUnjam();
                    break;
                }
                if (rightTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    rightTimer.reset();
                    rightState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (unjamRequested) {
                    startReverseUnjam();
                    break;
                }
                if (rightTimer.seconds() > STOP_DELAY_SEC) {
                    stopRightLauncher();
                    rightState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                if (rightTimer.seconds() > REVERSE_TIME_SEC) {
                    stopRightLauncher();
                    rightState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ---------------- Safe Getters ----------------
    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
    }

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
