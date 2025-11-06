package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Back + Fire (Left Power / Right PIDF)", group = "RoboAvengers")
public class BackFire2 extends LinearOpMode {

    // ------------- Hardware -------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private GoBildaPinpointDriver pinpoint;

    // ------------- Drive tuning -------------
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double kDrive = 0.02;

    // ------------- Launcher tuning -------------
    private static final double TARGET_VELOCITY = 2400; // ≈4400 RPM
    private static final int VEL_TOL = 200;
    private static final int READY_CYCLES = 4;

    // Left launcher uses open-loop power
    private static double LEFT_POWER_SCALE = 0.6; // default mid power

    // PIDF for right launcher
    private static final double R_P = 28.0;
    private static final double R_I = 0.0;
    private static final double R_D = 6.0;
    private static final double R_F = 12.0;

    private int readyCount = 0;

    private enum AutoState { BACK, FIRE, STOP, DONE }
    private AutoState state = AutoState.BACK;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Map hardware ---
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // --- Drive directions ---
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // --- Launcher setup ---
        if (leftLauncher != null) {
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // power mode
        }
        if (rightLauncher != null) {
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocityPIDFCoefficients(R_P, R_I, R_D, R_F);
        }

        // --- Pinpoint setup (optional) ---
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found — continuing without odometry.");
            pinpoint = null;
        }

        telemetry.addLine("READY: Back + Fire (Left Power / Right PIDF)");
        telemetry.addLine(String.format("Right PIDF: P=%.1f I=%.1f D=%.1f F=%.1f", R_P, R_I, R_D, R_F));
        telemetry.update();

        waitForStart();

        // --- Reapply PIDF (Expansion Hub quirk) ---
        if (rightLauncher != null)
            rightLauncher.setVelocityPIDFCoefficients(R_P, R_I, R_D, R_F);

        while (opModeIsActive() && state != AutoState.DONE) {
            if (pinpoint != null) pinpoint.update();

            switch (state) {
                case BACK:
                    telemetry.addLine("State: BACK");
                    if (moveToX(-48.0)) {
                        stopDrive();
                        sleep(500);
                        state = AutoState.FIRE;
                    }
                    break;

                case FIRE:
                    telemetry.addLine("State: FIRE");
                    startLaunchers();

                    if (waitUntilStable(3000)) {
                        if (intake != null) {
                            intake.setPower(0.5);
                            sleep(4000); // feed balls
                            intake.setPower(0);
                        }
                        stopLaunchers();
                        state = AutoState.STOP;
                    }
                    break;

                case STOP:
                    stopDrive();
                    state = AutoState.DONE;
                    break;
            }

            double leftVel = getLeftVelocity();
            double rightVel = rightLauncher != null ? rightLauncher.getVelocity() : 0;

            telemetry.addData("Left Power", LEFT_POWER_SCALE);
            telemetry.addData("Right Vel (tps)", rightVel);
            telemetry.addData("Δ (Left-Right)", leftVel - rightVel);
            telemetry.addData("Stable", isLaunchersStable());
            telemetry.update();
            idle();
        }

        stopLaunchers();
        stopDrive();
        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    // --- Movement ---
    private boolean moveToX(double targetXInches) {
        if (pinpoint == null) return true;
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double power = kDrive * error;
        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));
        setPower(power, power, power, power);
        return Math.abs(error) < 1.0;
    }

    // --- Launcher helpers ---
    private void startLaunchers() {
        double adjusted = getVoltageCompensatedVelocity(TARGET_VELOCITY);
        if (leftLauncher != null) leftLauncher.setPower(LEFT_POWER_SCALE);
        if (rightLauncher != null) rightLauncher.setVelocity(adjusted);
    }

    private boolean waitUntilStable(long timeoutMs) {
        readyCount = 0;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            if (isLaunchersStable()) return true;
            sleep(100);
        }
        return false;
    }

    private boolean isLaunchersStable() {
        if (rightLauncher == null) return false;
        double rt = rightLauncher.getVelocity();
        boolean inTol = Math.abs(rt - TARGET_VELOCITY) <= VEL_TOL;
        readyCount = inTol ? Math.min(READY_CYCLES, readyCount + 1) : 0;
        return readyCount >= READY_CYCLES;
    }

    private double getLeftVelocity() {
        if (leftLauncher == null) return 0;
        // estimate pseudo velocity (no encoder) based on power scale
        return LEFT_POWER_SCALE * TARGET_VELOCITY;
    }

    private void stopLaunchers() {
        if (leftLauncher != null) leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0;
        double currentVoltage = 12.0;
        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception ignored) {}
        return targetTicksPerSec * (nominalVoltage / currentVoltage);
    }

    // --- Utility ---
    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack != null) leftBack.setPower(bl);
        if (rightBack != null) rightBack.setPower(br);
    }

    private void stopDrive() { setPower(0, 0, 0, 0); }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}
