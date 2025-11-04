package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Back + Fire (Dual PIDF Tune)", group = "RoboAvengers")
public class BackFire2 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private GoBildaPinpointDriver pinpoint;

    // Motion control
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double kDrive = 0.02;

    // Launcher target velocity (ticks/sec)
    private static final double TARGET_VELOCITY = 8000; // ≈4400 RPM
    private static final int VEL_TOL = 150;
    private static final int READY_CYCLES = 5;

    // Left launcher PIDF
    private static final double L_P = 25.0;
    private static final double L_I = 0.0;
    private static final double L_D = 5.0;
    private static final double L_F = 12.0;

    // Right launcher PIDF
    private static final double R_P = 28.0;
    private static final double R_I = 0.0;
    private static final double R_D = 6.0;
    private static final double R_F = 12.5;

    private int readyCount = 0;

    private enum AutoState { BACK, FIRE, STOP, DONE }
    private AutoState state = AutoState.BACK;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // Drive directions
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Launcher setup
        if (leftLauncher != null) {
            leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLauncher.setVelocityPIDFCoefficients(L_P, L_I, L_D, L_F);
        }
        if (rightLauncher != null) {
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocityPIDFCoefficients(R_P, R_I, R_D, R_F);
        }

        // Pinpoint setup
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
            telemetry.addLine("Pinpoint not found");
            pinpoint = null;
        }

        telemetry.addLine("READY: Back + Fire (Dual PIDF Tune)");
        telemetry.addLine(String.format("Left PIDF:  P=%.1f I=%.1f D=%.1f F=%.1f", L_P, L_I, L_D, L_F));
        telemetry.addLine(String.format("Right PIDF: P=%.1f I=%.1f D=%.1f F=%.1f", R_P, R_I, R_D, R_F));
        telemetry.update();

        waitForStart();

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
                    telemetry.addLine("State: FIRE (Dual PIDF)");
                    startLaunchers();
                    waitUntilStable();

                    if (intake != null) intake.setPower(1.0);
                    sleep(4000);
                    if (intake != null) intake.setPower(0);

                    stopLaunchers();
                    state = AutoState.STOP;
                    break;

                case STOP:
                    stopDrive();
                    state = AutoState.DONE;
                    break;
            }

            telemetry.addData("Target (tps)", TARGET_VELOCITY);
            if (leftLauncher != null) {
                double leftErr = ((leftLauncher.getVelocity() - TARGET_VELOCITY) / TARGET_VELOCITY) * 100.0;
                telemetry.addData("Left Vel", "%.0f (err %.1f%%)", leftLauncher.getVelocity(), leftErr);
            }
            if (rightLauncher != null) {
                double rightErr = ((rightLauncher.getVelocity() - TARGET_VELOCITY) / TARGET_VELOCITY) * 100.0;
                telemetry.addData("Right Vel", "%.0f (err %.1f%%)", rightLauncher.getVelocity(), rightErr);
            }
            telemetry.addData("Stable", isLaunchersStable());
            telemetry.update();
            idle();
        }

        stopLaunchers();
        stopDrive();
        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    // Movement
    private boolean moveToX(double targetXInches) {
        if (pinpoint == null) return true;
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double power = kDrive * error;
        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));
        setPower(power, power, power, power);
        return Math.abs(error) < 1.0;
    }

    // Launcher
    private void startLaunchers() {
        double adjusted = getVoltageCompensatedVelocity(TARGET_VELOCITY);
        if (leftLauncher != null) leftLauncher.setVelocity(adjusted);
        if (rightLauncher != null) rightLauncher.setVelocity(adjusted);
    }

    private void waitUntilStable() {
        readyCount = 0;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            telemetry.addData("Left Vel", leftLauncher != null ? leftLauncher.getVelocity() : 0);
            telemetry.addData("Right Vel", rightLauncher != null ? rightLauncher.getVelocity() : 0);
            telemetry.addData("Stable", isLaunchersStable());
            telemetry.update();
            sleep(100);
        }
    }

    private boolean isLaunchersStable() {
        if (leftLauncher == null || rightLauncher == null) return false;
        double lt = leftLauncher.getVelocity();
        double rt = rightLauncher.getVelocity();
        boolean inTol = Math.abs(lt - TARGET_VELOCITY) <= VEL_TOL &&
                Math.abs(rt - TARGET_VELOCITY) <= VEL_TOL;
        readyCount = inTol ? Math.min(READY_CYCLES, readyCount + 1) : 0;
        return readyCount >= READY_CYCLES;
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
        } catch (Exception e) {
            // ignore
        }
        return targetTicksPerSec * (nominalVoltage / currentVoltage);
    }

    // Utility
    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack != null) leftBack.setPower(bl);
        if (rightBack != null) rightBack.setPower(br);
    }

    private void stopDrive() {
        setPower(0, 0, 0, 0);
    }

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
