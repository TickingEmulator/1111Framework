package frc.robot.Library;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/*
 * MotorConfigurator, used to configure the SparkMax to prevent code redundency
 * Enjoy, by Brayden
 */
public class SparkConfigurator {

    private SparkMaxConfig sparkMaxConfig;

    /**
     * Constructs a MotorConfigurator using the provided SparkMaxConfig.
     *
     * @param sparkMaxConfig the SparkMax configuration object to be configured
     */
    public SparkConfigurator(SparkMaxConfig sparkMaxConfig) {
        this.sparkMaxConfig = sparkMaxConfig;
    }

    /**
     * Applies basic configurations such as idle mode, inversion, ramp rate, and current limit.
     *
     * @param idleMode          the idle mode to set
     * @param inverted          whether the motor output is inverted
     * @param isOpenRamp        if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate          the ramp rate value to apply
     * @param smartCurrentLimit the smart current limit to apply
     */
    private void applyBasicConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit) {
        sparkMaxConfig.idleMode(idleMode);
        sparkMaxConfig.inverted(inverted);
        if (isOpenRamp) {
            sparkMaxConfig.openLoopRampRate(rampRate);
        } else {
            sparkMaxConfig.closedLoopRampRate(rampRate);
        }
        sparkMaxConfig.smartCurrentLimit(smartCurrentLimit);
        /*
         * All signals should remain on to ensure accessibility to information when required.
         */
        sparkMaxConfig.signals.absoluteEncoderPositionAlwaysOn(true);
        sparkMaxConfig.signals.absoluteEncoderVelocityAlwaysOn(true);
        sparkMaxConfig.signals.faultsAlwaysOn(true);
        sparkMaxConfig.signals.iAccumulationAlwaysOn(true);
        sparkMaxConfig.signals.primaryEncoderPositionAlwaysOn(true);
        sparkMaxConfig.signals.primaryEncoderVelocityAlwaysOn(true);
        sparkMaxConfig.signals.warningsAlwaysOn(true);
    }

    /**
     * Applies basic configurations.
     *
     * @param idleMode          the idle mode to set
     * @param inverted          whether the motor output is inverted
     * @param isOpenRamp        if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate          the ramp rate value to apply
     * @param smartCurrentLimit the smart current limit to apply
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
    }

    /**
     * Applies basic configurations and sets this motor to follow another.
     *
     * @param idleMode          the idle mode to set
     * @param inverted          whether the motor output is inverted
     * @param isOpenRamp        if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate          the ramp rate value to apply
     * @param smartCurrentLimit the smart current limit to apply
     * @param canLeader         the SparkMax to follow
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, SparkMax canLeader) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        sparkMaxConfig.follow(canLeader, inverted);
    }

    /**
     * Applies basic configurations with encoder settings for position conversion.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param positionConversionFactor the conversion factor for encoder position
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double positionConversionFactor) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.positionConversionFactor(positionConversionFactor);
        } else {
            sparkMaxConfig.encoder.positionConversionFactor(positionConversionFactor);
        }
    }

    /**
     * Applies basic configurations with encoder settings for velocity conversion.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param velocityConversionFactor the conversion factor for encoder velocity
     * @param controlType           the control type for the encoder
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double velocityConversionFactor, ControlType controlType) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.velocityConversionFactor(velocityConversionFactor);
        } else {
            sparkMaxConfig.encoder.velocityConversionFactor(velocityConversionFactor);
        }
        // Note: controlType is provided but not used in this snippet. Use it as needed.
    }

    /**
     * Applies basic configurations with both encoder (position and velocity) settings.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param positionConversionFactor the conversion factor for encoder position
     * @param velocityConversionFactor the conversion factor for encoder velocity
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double positionConversionFactor, double velocityConversionFactor) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.positionConversionFactor(positionConversionFactor);
            sparkMaxConfig.absoluteEncoder.velocityConversionFactor(velocityConversionFactor);
        } else {
            sparkMaxConfig.encoder.positionConversionFactor(positionConversionFactor);
            sparkMaxConfig.encoder.velocityConversionFactor(velocityConversionFactor);
        }
    }

    /**
     * Applies basic, encoder, and PID configurations.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param positionConversionFactor the conversion factor for encoder position
     * @param positionWrappingEnabled whether position wrapping is enabled
     * @param positionMinInput      the minimum input value for position wrapping
     * @param positionMaxInput      the maximum input value for position wrapping
     * @param P                     the proportional gain for PID
     * @param I                     the integral gain for PID
     * @param D                     the derivative gain for PID
     * @param FF                    the feedforward gain for PID
     * @param maxAcceleration       the maximum acceleration allowed in closed-loop motion
     * @param maxVelocity           the maximum velocity allowed in closed-loop motion
     * @param errorTolerance        the allowed closed-loop error
     * @param modeType the motion type of the motor
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double positionConversionFactor, boolean positionWrappingEnabled,
        double positionMinInput, double positionMaxInput,  double P, double I, double D, double FF, double maxAcceleration, double maxVelocity, double errorTolerance, MAXMotionPositionMode modeType) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.positionConversionFactor(positionConversionFactor);
        } else {
            sparkMaxConfig.encoder.positionConversionFactor(positionConversionFactor);
        }
        sparkMaxConfig.closedLoop.outputRange(-1, 1);
        sparkMaxConfig.closedLoop.positionWrappingEnabled(positionWrappingEnabled);
        sparkMaxConfig.closedLoop.positionWrappingInputRange(positionMinInput, positionMaxInput);
        sparkMaxConfig.closedLoop.pidf(P, I, D, FF)
            .maxMotion.maxAcceleration(maxAcceleration)
            .maxVelocity(maxVelocity)
            .allowedClosedLoopError(errorTolerance)
            .positionMode(modeType);
    }

    /**
     * Applies basic, encoder, and PID configurations for velocity control.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param velocityConversionFactor the conversion factor for encoder velocity
     * @param controlType           the control type for the encoder
     * @param positionWrappingEnabled whether position wrapping is enabled
     * @param positionMinInput      the minimum input value for position wrapping
     * @param positionMaxInput      the maximum input value for position wrapping
     * @param P                     the proportional gain for PID
     * @param I                     the integral gain for PID
     * @param D                     the derivative gain for PID
     * @param FF                    the feedforward gain for PID
     * @param maxAcceleration       the maximum acceleration allowed in closed-loop motion
     * @param maxVelocity           the maximum velocity allowed in closed-loop motion
     * @param errorTolerance        the allowed closed-loop error
     * @param modeType the motion type of the motor
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double velocityConversionFactor, ControlType controlType,
        boolean positionWrappingEnabled, double positionMinInput, double positionMaxInput, double P, double I, double D, double FF,  double maxAcceleration, double maxVelocity, double errorTolerance, MAXMotionPositionMode modeType) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.velocityConversionFactor(velocityConversionFactor);
        } else {
            sparkMaxConfig.encoder.velocityConversionFactor(velocityConversionFactor);
        }
        sparkMaxConfig.closedLoop.outputRange(-1, 1);
        sparkMaxConfig.closedLoop.positionWrappingEnabled(positionWrappingEnabled);
        sparkMaxConfig.closedLoop.positionWrappingInputRange(positionMinInput, positionMaxInput);
        sparkMaxConfig.closedLoop.pidf(P, I, D, FF).maxMotion.maxAcceleration(maxAcceleration).maxVelocity(maxVelocity).allowedClosedLoopError(errorTolerance).positionMode(modeType);
    }
    /**
     * Applies basic, encoder, and PID configurations for both position and velocity.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param positionConversionFactor the conversion factor for encoder position
     * @param velocityConversionFactor the conversion factor for encoder velocity
     * @param positionWrappingEnabled whether position wrapping is enabled
     * @param positionMinInput      the minimum input value for position wrapping
     * @param positionMaxInput      the maximum input value for position wrapping
     * @param P                     the proportional gain for PID
     * @param I                     the integral gain for PID
     * @param D                     the derivative gain for PID
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double positionConversionFactor, double velocityConversionFactor,
        boolean positionWrappingEnabled, double positionMinInput, double positionMaxInput, double P, double I, double D) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.positionConversionFactor(positionConversionFactor);
            sparkMaxConfig.absoluteEncoder.velocityConversionFactor(velocityConversionFactor);
        } else {
            sparkMaxConfig.encoder.positionConversionFactor(positionConversionFactor);
            sparkMaxConfig.encoder.velocityConversionFactor(velocityConversionFactor);
        }
        sparkMaxConfig.closedLoop.outputRange(-1, 1);
        sparkMaxConfig.closedLoop.positionWrappingEnabled(positionWrappingEnabled);
        sparkMaxConfig.closedLoop.positionWrappingInputRange(positionMinInput, positionMaxInput);
        sparkMaxConfig.closedLoop.pid(P, I, D);
    }

    /**
     * Applies basic, encoder, and PID configurations for both position and velocity.
     *
     * @param idleMode              the idle mode to set
     * @param inverted              whether the motor output is inverted
     * @param isOpenRamp            if true, applies open-loop ramp rate; otherwise, closed-loop ramp rate
     * @param rampRate              the ramp rate value to apply
     * @param smartCurrentLimit     the smart current limit to apply
     * @param isAbsolute            if true, configures the absolute encoder; otherwise, the relative encoder
     * @param positionConversionFactor the conversion factor for encoder position
     * @param velocityConversionFactor the conversion factor for encoder velocity
     * @param positionWrappingEnabled whether position wrapping is enabled
     * @param positionMinInput      the minimum input value for position wrapping
     * @param positionMaxInput      the maximum input value for position wrapping
     * @param P                     the proportional gain for PID
     * @param I                     the integral gain for PID
     * @param D                     the derivative gain for PID
     * @param FF                    the feedforward gain for PID
     * @param maxAcceleration       the maximum acceleration allowed in closed-loop motion
     * @param maxVelocity           the maximum velocity allowed in closed-loop motion
     * @param errorTolerance        the allowed closed-loop error
     * @param modeType the motion type of the motor
     */
    public void applyConfigurations(IdleMode idleMode, boolean inverted, boolean isOpenRamp, double rampRate, int smartCurrentLimit, boolean isAbsolute, double positionConversionFactor, double velocityConversionFactor,
        boolean positionWrappingEnabled, double positionMinInput, double positionMaxInput, double P, double I, double D, double FF, double maxAcceleration, double maxVelocity, double errorTolerance, MAXMotionPositionMode modeType) {
        applyBasicConfigurations(idleMode, inverted, isOpenRamp, rampRate, smartCurrentLimit);
        if (isAbsolute) {
            sparkMaxConfig.absoluteEncoder.positionConversionFactor(positionConversionFactor);
            sparkMaxConfig.absoluteEncoder.velocityConversionFactor(velocityConversionFactor);
        } else {
            sparkMaxConfig.encoder.positionConversionFactor(positionConversionFactor);
            sparkMaxConfig.encoder.velocityConversionFactor(velocityConversionFactor);
        }
        sparkMaxConfig.closedLoop.outputRange(-1, 1);
        sparkMaxConfig.closedLoop.positionWrappingEnabled(positionWrappingEnabled);
        sparkMaxConfig.closedLoop.positionWrappingInputRange(positionMinInput, positionMaxInput);
        sparkMaxConfig.closedLoop.pidf(P, I, D, FF).maxMotion.maxAcceleration(maxAcceleration).maxVelocity(maxVelocity).allowedClosedLoopError(errorTolerance).positionMode(modeType);
    }
    public SparkMaxConfig getApplied(){
        return this.sparkMaxConfig;
    }
}
