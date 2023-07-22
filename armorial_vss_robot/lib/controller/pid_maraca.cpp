#include "pid_maraca.h"

//**********************************
// Constructor functions
//**********************************
PIDMaraca::PIDMaraca()
{
    init();
}

PIDMaraca::PIDMaraca(const float& P, const float& I, const float& D)
{
    init();
    _PP = P;
    _II = I;
    _DD = D;
}

PIDMaraca::PIDMaraca(const float& P, const float& I, const float& D, const float& F)
{
    init();
    _PP = P;
    _II = I;
    _DD = D;
    _FF = F;
}

PIDMaraca::PIDMaraca(std::tuple<const float&, const float&, const float&> pid)
    : PIDMaraca(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid))
{
}

PIDMaraca::PIDMaraca(std::tuple<const float&, const float&, const float&, const float&> pidf)
    : PIDMaraca(std::get<0>(pidf), std::get<1>(pidf), std::get<2>(pidf), std::get<3>(pidf))
{
}

void PIDMaraca::init()
{
    _PP = 0;
    _II = 0;
    _DD = 0;
    _FF = 0;

    _maxIOutput = 0;
    _maxError = 0;
    _errorSum = 0;
    _maxOutput = 0;
    _minOutput = 0;
    _setpoint = 0;
    _lastActual = 0;
    _firstRun = true;
    _reversed = false;
    _outputRampRate = 0;
    _lastOutput = 0;
    _outputFilter = 0;
    _setpointRange = 0;
}

//**********************************
// Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter. <br>
 * this->responds quicly to changes in setpoint, and provides most of the initial driving force
 * to make corrections. <br>
 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
 * For position based controllers, this->is the first parameter to tune, with I second. <br>
 * For rate controlled systems, this->is often the second after F.
 *
 * @param P Proportional gain. Affects output according to
 * <b>output+=P*(setpoint-current_value)</b>
 */
void PIDMaraca::setP(const float& P)
{
    _PP = P;
    checkSigns();
}

/**
 * Changes the I parameter <br>
 * this->is used for overcoming disturbances, and ensuring that the controller always gets to
 * the control mode. Typically tuned second for "Position" based modes, and third for "Rate" or
 * continuous based modes. <br> Affects output through <b>output+=previous_errors*Igain
 * ;previous_errors+=current_error</b>
 *
 * @see {@link #setMaxIOutput(float) setMaxIOutput} for how to restrict
 *
 * @param i New gain value for the Integral term
 */
void PIDMaraca::setI(const float& I)
{
    if (_II != 0) {
        _errorSum = _errorSum * _II / I;
    }
    if (_maxIOutput != 0) {
        _maxError = _maxIOutput / I;
    }
    _II = I;
    checkSigns();
    /* Implementation note:
     * this->Scales the accumulated error to avoid output errors.
     * As an example doubling the I term cuts the accumulated error in half, which results in
     * the output change due to the I term constant during the transition.
     *
     */
}

void PIDMaraca::setD(const float& D)
{
    _DD = D;
    checkSigns();
}

/**Configure the FeedForward parameter. <br>
 * this->is excellent for Velocity, rate, and other	continuous control modes where you can
 * expect a rough output value based solely on the setpoint.<br>
 * Should not be used in "position" based control modes.
 *
 * @param f Feed forward gain. Affects output according to <b>output+=F*Setpoint</b>;
 */
void PIDMaraca::setF(const float& F)
{
    _FF = F;
    checkSigns();
}

/** Create a new PID object.
 * @param P Proportional gain. Large if large difference between setpoint and target.
 * @param I Integral gain.	Becomes large if setpoint cannot reach target quickly.
 * @param D Derivative gain. Responds quickly to large changes in error. Small values prevents P
 * and I terms from causing overshoot.
 */
void PIDMaraca::setPID(const float& P, const float& I, const float& D)
{
    _PP = P;
    _II = I;
    _DD = D;
    checkSigns();
}

void PIDMaraca::setPID(const float& P, const float& I, const float& D, const float& F)
{
    _PP = P;
    _II = I;
    _DD = D;
    _FF = F;
    checkSigns();
}

/**Set the maximum output value contributed by the I component of the system
 * this->can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void PIDMaraca::setMaxIOutput(const float& maxIOutput)
{
    /* Internally maxError and Izone are similar, but scaled for different purposes.
     * The maxError is generated for simplifying math, since calculations against
     * the max error are far more common than changing the I term or Izone.
     */
    _maxIOutput = maxIOutput;
    if (_II != 0) {
        _maxError = _maxIOutput / _II;
    }
}

/**Specify a maximum output. If a single parameter is specified, the minimum is
 * set to (-maximum).
 * @param output
 */
void PIDMaraca::setOutputLimits(const float& outputLimit)
{
    setOutputLimits(-outputLimit, outputLimit);
}

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void PIDMaraca::setOutputLimits(const float& minOutput, const float& maxOutput)
{
    if (maxOutput < minOutput)
        return;
    _maxOutput = maxOutput;
    _minOutput = minOutput;

    // Ensure the bounds of the I term are within the bounds of the allowable output swing
    if (_maxIOutput == 0 || _maxIOutput > (maxOutput - minOutput)) {
        setMaxIOutput(maxOutput - minOutput);
    }
}

/** Set the operating direction of the PID controller
 * @param reversed Set true to reverse PID output
 */
void PIDMaraca::setDirection(const bool& reversed)
{
    this->_reversed = reversed;
}

//**********************************
// Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void PIDMaraca::setSetpoint(const float& setpoint)
{
    this->_setpoint = setpoint;
}

/** Calculate the PID value needed to hit the target setpoint.
 * Automatically re-calculates the output at each call.
 * @param actual The monitored value
 * @param target The target value
 * @return calculated output value for driving the actual to the target
 */
float PIDMaraca::getOutput(const float& actual, float setpoint)
{
    float output;
    float Poutput;
    float Ioutput;
    float Doutput;
    float Foutput;

    this->_setpoint = setpoint;

    // Ramp the setpoint used for calculations if user has opted to do so
    if (_setpointRange != 0) {
        setpoint = clamp(setpoint, actual - _setpointRange, actual + _setpointRange);
    }

    // Do the simple parts of the calculations
    float error = setpoint - actual;

    // Calculate F output. Notice, this->depends only on the setpoint, and not the error.
    Foutput = _FF * setpoint;

    // Calculate P term
    Poutput = _PP * error;

    // If this->is our first time running this-> we don't actually _have_ a previous input or
    // output. For sensor, sanely assume it was exactly where it is now. For last output, we can
    // assume it's the current time-independent outputs.
    if (_firstRun) {
        _lastActual = actual;
        _lastOutput = Poutput + Foutput;
        _firstRun = false;
    }

    // Calculate D Term
    // Note, this->is negative. this->actually "slows" the system if it's doing
    // the correct thing, and small values helps prevent output spikes and overshoot

    Doutput = -_DD * (actual - _lastActual) * 60;
    _lastActual = actual;

    // The Iterm is more complex. There's several things to factor in to make it easier to deal
    // with.
    // 1. maxIoutput restricts the amount of output contributed by the Iterm.
    // 2. prevent windup by not increasing errorSum if we're already running against our max
    // Ioutput
    // 3. prevent windup by not increasing errorSum if output is output=maxOutput
    Ioutput = _II * _errorSum / 60;
    if (_maxIOutput != 0) {
        Ioutput = clamp(Ioutput, -_maxIOutput, _maxIOutput);
    }

    // And, finally, we can just add the terms up
    output = Foutput + Poutput + Ioutput + Doutput;

    // Figure out what we're doing with the error.
    if (_minOutput != _maxOutput && !bounded(output, _minOutput, _maxOutput)) {
        _errorSum = error;
        // reset the error sum to a sane level
        // Setting to current error ensures a smooth transition when the P term
        // decreases enough for the I term to start acting upon the controller
        // From that point the I term will build up as would be expected
    } else if (_outputRampRate != 0 &&
               !bounded(output, _lastOutput - _outputRampRate, _lastOutput + _outputRampRate)) {
        _errorSum = error;
    } else if (_maxIOutput != 0) {
        _errorSum = clamp(_errorSum + error, -_maxError, _maxError);
        // In addition to output limiting directly, we also want to prevent I term
        // buildup, so restrict the error directly
    } else {
        _errorSum += error;
    }

    // Restrict output to our specified output and ramp limits
    if (_outputRampRate != 0) {
        output = clamp(output, _lastOutput - _outputRampRate, _lastOutput + _outputRampRate);
    }
    if (_minOutput != _maxOutput) {
        output = clamp(output, _minOutput, _maxOutput);
    }
    if (_outputFilter != 0) {
        output = _lastOutput * _outputFilter + output * (1 - _outputFilter);
    }

    _lastOutput = output;
    return output;
}

/**
 * Calculates the PID value using the last provided setpoint and actual valuess
 * @return calculated output value for driving the actual to the target
 */
float PIDMaraca::getOutput()
{
    return getOutput(_lastActual, _setpoint);
}

/**
 *
 * @param actual
 * @return calculated output value for driving the actual to the target
 */
float PIDMaraca::getOutput(const float& actual)
{
    return getOutput(actual, _setpoint);
}

/**
 * Resets the controller. this->erases the I term buildup, and removes D gain on the next loop.
 */
void PIDMaraca::reset()
{
    _firstRun = true;
    _errorSum = 0;
}

/**Set the maximum rate the output can increase per cycle.
 * @param rate
 */
void PIDMaraca::setOutputRampRate(const float& rate)
{
    _outputRampRate = rate;
}

/** Set a limit on how far the setpoint can be from the current position
 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range.
 * <br>this->limits the reactivity of P term, and restricts impact of large D term
 * during large setpoint adjustments. Increases lag and I term if range is too small.
 * @param range
 */
void PIDMaraca::setSetpointRange(const float& range)
{
    _setpointRange = range;
}

/**Set a filter on the output to reduce sharp oscillations. <br>
 * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I
 * values. Uses an exponential rolling sum filter, according to a simple <br>
 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
 * @param output valid between [0..1), meaning [current output only.. historical output only)
 */
void PIDMaraca::setOutputFilter(const float& strength)
{
    if (strength == 0 || bounded(strength, 0, 1)) {
        _outputFilter = strength;
    }
}

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise
 */
float PIDMaraca::clamp(const float& value, const float& min, const float& max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool PIDMaraca::bounded(const float& value, const float& min, const float& max)
{
    return (min < value) && (value < max);
}

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed value
 */
void PIDMaraca::checkSigns()
{
    if (_reversed) {  // all values should be below zero
        if (_PP > 0)
            _PP *= -1;
        if (_II > 0)
            _II *= -1;
        if (_DD > 0)
            _DD *= -1;
        if (_FF > 0)
            _FF *= -1;
    } else {  // all values should be above zero
        if (_PP < 0)
            _PP *= -1;
        if (_II < 0)
            _II *= -1;
        if (_DD < 0)
            _DD *= -1;
        if (_FF < 0)
            _FF *= -1;
    }
}

void PIDMaraca::setPID(std::tuple<const float&, const float&, const float&> pid)
{
    this->setPID(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
}

void PIDMaraca::setPID(std::tuple<const float&, const float&, const float&> pid, const float& f)
{
    this->setPID(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid), f);
}