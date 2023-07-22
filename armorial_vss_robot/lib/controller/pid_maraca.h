#ifndef ARMORIAL_SUASSUNA_PID_MARACA
#define ARMORIAL_SUASSUNA_PID_MARACA

#include <tuple>

class PIDMaraca
{
public:
    PIDMaraca();

    PIDMaraca(const float& P, const float& I, const float& D);
    PIDMaraca(const float& P, const float& I, const float& D, const float& F);
    PIDMaraca(std::tuple<const float&, const float&, const float&>);
    PIDMaraca(std::tuple<const float&, const float&, const float&, const float&>);

    void setP(const float& P);
    void setI(const float& I);
    void setD(const float& D);
    void setF(const float& F);
    void setPID(const float& P, const float& I, const float& D);
    void setPID(std::tuple<const float&, const float&, const float&>);
    void setPID(std::tuple<const float&, const float&, const float&>, const float&);

    void setPID(const float& P, const float& I, const float& D, const float& F);
    void setMaxIOutput(const float& maxIOutput);
    void setOutputLimits(const float& outputLimit);
    void setOutputLimits(const float& minOutput, const float& maxOutput);
    void setDirection(const bool& reveresed);
    void setSetpoint(const float& setpoint);
    void reset();
    void setOutputRampRate(const float& rate);
    void setSetpointRange(const float& range);
    void setOutputFilter(const float& strength);
    float getOutput();
    float getOutput(const float& actual);
    float getOutput(const float& actual, float setpoint);

private:
    static float clamp(const float&, const float&, const float&);
    static bool bounded(const float&, const float&, const float&);
    void checkSigns();
    void init();

    float _PP;
    float _II;
    float _DD;
    float _FF;

    float _maxIOutput;
    float _maxError;
    float _errorSum;

    float _maxOutput;
    float _minOutput;

    float _setpoint;

    float _lastActual;

    bool _firstRun;
    bool _reversed;

    float _outputRampRate;
    float _lastOutput;

    float _outputFilter;

    float _setpointRange;
};

#endif /* ARMORIAL_SUASSUNA_PID_MARACA */
