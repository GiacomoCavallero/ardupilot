#include <OC_NMEA2k/oc_calibration.h>
#include <AP_Math/AP_Math.h>
#include <map>

double calCompass(double h)
{
    // Setup deviation table
    // Eventually from file
    std::map<int,double> devTable;
    devTable.insert({  0, 1});
    devTable.insert({ 30,-1});
    devTable.insert({ 60, 2});
    devTable.insert({ 90, 4});
    devTable.insert({120, 0});
    devTable.insert({180,-3});
    devTable.insert({240,-1});
    devTable.insert({280, 1});
    devTable.insert({330, 3});

    // Lookup table and interpolate
    int h1, h2;
    double c1, c2;

    auto it = devTable.lower_bound(h);

    if (it == devTable.begin()) {
        h1 = std::prev(devTable.end())->first - 360;
        c1 = std::prev(devTable.end())->second;
        h2 = it->first;
        c2 = it->second;
    }
    else if (it == devTable.end()) {
        h1 = std::prev(devTable.end())->first;
        c1 = std::prev(devTable.end())->second;
        h2 = devTable.begin()->first + 360;
        c2 = devTable.begin()->second;
    }
    else {
        h1 = std::prev(it)->first;
        c1 = std::prev(it)->second;
        h2 = it->first;
        c2 = it->second;
    }

    double c = linInterp(h1,h2,c1,c2,h);
    return wrap_360(h + c);
}

double linInterp(double h1, double h2, double c1, double c2, double h)
{
    double c = c1;

    if (fpclassify(h2 - h1) != FP_ZERO) {
        c =  c1 + (c2-c1) * (h-h1) / (h2-h1);
    }
    return c;
    
}