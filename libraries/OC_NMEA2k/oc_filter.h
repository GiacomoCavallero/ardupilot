#pragma once

#include <deque>
#include <chrono>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

template <typename T>
class FiltVar
{
    // Class for variables to be filtered. Adds timestamp to data
private:
    std::chrono::system_clock::time_point timestamp_;
    T var_;

public:
    FiltVar(T var);
    FiltVar();
    void setVar(T var) { var_ = var; }
    std::chrono::system_clock::time_point timestamp() { return timestamp_; }
    T var() { return var_; }
};

template <typename T>
class FiltBoxcar
{
    // Boxcar filtering class for scalar values
public:
    FiltBoxcar(AP_Float* secs) : bctime{secs} {} // Creates empty deque and sets up parameters
    T filterPoint(T point);                                    // Adds variable, trims old and returns filtered value

private:
    AP_Float* bctime;
    std::deque<FiltVar<T>> sample;
};

template <typename T>
class FiltBoxcarAng : FiltBoxcar<T>
{
    // Boxcar filtering class for angular data
    // mod = 359: returns angle in 0 to 360 range
    // mod = 179: returns angle in -180 to 180 range
public:
    FiltBoxcarAng(AP_Float* secs, int mod = 359) : FiltBoxcar<T>::bctime{secs}, mod_{mod} {} // Creates empty deque and sets up parameters
    T filterPoint(T angle);

private:
    int mod_;
};

template <typename T>
Vector2<T> dToUv(T d)
{
    return Vector2<T>(-sin(radians(d)), -cos(radians(d)));
}

template <typename T>
class FiltExp
{
    // Exponential filtering class for scalar values
public:
    FiltExp(AP_Float* secs) : tau{secs} {} // Creates initial oldPoint and sets up parameters
    T filterPoint(T point);                // Adds new data point and returns filtered value

private:
    AP_Float* tau;
    FiltVar<T> oldPoint;
};

template <typename T>
class FiltExpAng : FiltExp<Vector2<T>>
{
    // Exponential filtering class for angular data
    // mod = 359: returns angle in 0 to 360 range
    // mod = 179: returns angle in -180 to 180 range
public:
    FiltExpAng(AP_Float* secs, int mod = 359) : FiltExp<Vector2<T>>(secs), mod_{mod} {} // Creates initlal oldPoint and sets up parameters
    T filterPoint(T angle);

private:
    int mod_;
};

template <typename T>
class FiltExpNl
{
    // Non-linear exponential filtering class for scalar values
    // Damping reduces to -1 by 8 * bound
public:
    FiltExpNl(AP_Float* secs, AP_Float* bound_) : tau{secs}, bound{bound_} {} // Creates initial lastPoint and sets up parameters
    T filterPoint(T point);                                           // Adds new data point and returns filtered value

private:
    AP_Float* tau;
    AP_Float* bound;
    FiltVar<T> oldPoint;
};

template <typename T>
class FiltExpNlAng : FiltExpNl<Vector2<T>>
{
    // Non-linear exponential filtering class for angular data
    // mod = 359: returns angle in 0 to 360 range
    // mod = 179: returns angle in -180 to 180 range
public:
    FiltExpNlAng(AP_Float* secs, AP_Float* bound_, int mod = 359) : FiltExpNl<Vector2<T>>(secs,bound_), mod_{mod} {} // Creates initial lastPoint and sets up parameters
    T filterPoint(T angle);

private:
    int mod_;
};


// Overload of abs so we can process vector2
// Returns absolute value of angle (not magnitude)
template <typename T>
T abs(Vector2<T> vec)
{
    //vec.angle should use atan2 which will give +=180 but wrap anyway
    return abs(wrap_180(ToDeg(vec.angle())));
}



//Definitions - templates so put in header for visibility

template <typename T>
FiltVar<T>::FiltVar(T var)
{
    // Construct a timestamped var
    var_ = var;
    timestamp_ = std::chrono::system_clock::now();
}

template <typename T>
FiltVar<T>::FiltVar()
{
    // Construct a timestamp, Leave var_ default initialized
    timestamp_ = std::chrono::system_clock::now();
}

template <typename T>
T FiltBoxcar<T>::filterPoint(T point)
{
    // Add new point to deque
    sample.push_front(FiltVar<T>(point));

    // Trim off old points
    while (!sample.empty() &&
           std::chrono::duration<double>(point.timestamp() - sample.back().timestamp()).count() > *bctime)
    {
        sample.pop_back();
    }

    // Return boxcar averaged value
    T acc;
    for (auto i : sample)
    {
        acc += i.var();
    }
    size_t n = sample.size();

    return n ? acc / n : 0;
}

template <typename T>
T FiltBoxcarAng<T>::filterPoint(T angle)
{
    // Convert directional value to u/v, filter, convert back and wrap to appropiate range
    Vector2<T> dataPoint(dToUv(angle));
    Vector2<T> filtVec = FiltBoxcar<T>::filterPoint(dataPoint);

    T angleFiltered = ToDeg(filtVec.angle());
    return mod_ == 180 ? wrap_180(angleFiltered) : wrap_360(angleFiltered);
}

template <typename T>
T FiltExp<T>::filterPoint(T point)
{
    // Create new datapoint and calculate smoothing constant based on time difference
    FiltVar<T> newPoint(point);
    double dTime = std::chrono::duration<double>(newPoint.timestamp() - oldPoint.timestamp()).count();
    double a = 0;
    if (fpclassify(*tau) != FP_ZERO)
        a = std::exp(-dTime / (*tau));

    // Return filtered value and store for next time
    T newVal = oldPoint.var() * a + newPoint.var() * (1 - a);
    newPoint.setVar(newVal);
    oldPoint = newPoint;
    return newVal;
}

template <typename T>
T FiltExpAng<T>::filterPoint(T angle)
{
    // Convert directional value to u/v, filter, convert back and wrap to appropiate range
    Vector2<T> dataPoint(dToUv(angle));
    Vector2<T> filtVec = this->FiltExp<Vector2<T>>::filterPoint(dataPoint);

    T angleFiltered = ToDeg(filtVec.angle());
    return mod_ == 180 ? wrap_180(angleFiltered) : wrap_360(angleFiltered);
}

template <typename T>
T FiltExpNl<T>::filterPoint(T point)
{
    // Create new datapoint and calculate smoothing constant based on time difference 
    // If new point is outside bound from oldPoint (units per second), reduce smoothing constant

    FiltVar<T> newPoint(point);
    double dTime = std::chrono::duration<double>(newPoint.timestamp() - oldPoint.timestamp()).count();

    double a = 0;
    if (fpclassify(*tau) != FP_ZERO)
        a = std::exp(-dTime / *(tau));

    // Calculate non-linear factor
    double bFac = 0;
    if (fpclassify(*bound) != FP_ZERO) {
        T dVar = newPoint.var() - oldPoint.var();
        bFac = abs((dVar / dTime) / (*bound));
    }

    if (bFac > 1)
    {
        // Reduce damping to 0 by the time we are 8 times bounds - for faster
        // response during manouvers
        a = std::max<double>(0, (-a / 7) * (bFac - 1) + a);
    }

    // Return exponential filtered value and store for next time
    T newVal = oldPoint.var() * a + newPoint.var() * (1 - a);
    newPoint.setVar(newVal);
    oldPoint = newPoint;
    return newVal;
}

template <typename T>
T FiltExpNlAng<T>::filterPoint(T angle)
{
    // Convert directional value to u/v, filter, convert back and wrap to appropiate range
    Vector2<T> dataPoint(dToUv(angle));
    Vector2<T> filtVec = this->FiltExpNl<Vector2<T>>::filterPoint(dataPoint);

    T angleFiltered = ToDeg(filtVec.angle());
    return mod_ == 180 ? wrap_180(angleFiltered) : wrap_360(angleFiltered);
}
