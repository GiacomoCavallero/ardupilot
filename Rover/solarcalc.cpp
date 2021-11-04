#include <math.h>
//#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <sys/time.h>
#define Pi (3.141592654)

float calcTimeJulianCent(float jd)
{
    float T = (jd - 2451545.0)/36525.0;
    return T;
}

float calcJDFromJulianCent(float t)
{
    float JD = t * 36525.0 + 2451545.0;
    return JD;
}

int isLeapYear(int yr)
{
    return ((yr % 4 == 0 && yr % 100 != 0) || yr % 400 == 0);
}

float calcDoyFromJD(float jd)
{
    float z = floor(jd + 0.5);
    float f = (jd + 0.5) - z;
    float A = 0;
    if (z < 2299161) {
        A = z;
    } else {
        float alpha = floor((z - 1867216.25)/36524.25);
        A = z + 1 + alpha - floor(alpha/4);
    }
    float B = A + 1524;
    float C = floor((B - 122.1)/365.25);
    float D = floor(365.25 * C);
    float E = floor((B - D)/30.6001);
    float day = B - D - floor(30.6001 * E) + f;
    float month = (E < 14) ? E - 1 : E - 13;
    float year = (month > 2) ? C - 4716 : C - 4715;

    float k = (isLeapYear(year) ? 1 : 2);
    float doy = floor((275 * month)/9) - k * floor((month + 9)/12) + day -30;
    return doy;
}


float radToDeg(float angleRad)
{
    return (180.0 * angleRad / Pi);
}

float degToRad(float angleDeg)
{
    return (Pi * angleDeg / 180.0);
}

float calcGeomMeanLongSun(float t)
{
    float L0 = 280.46646 + t * (36000.76983 + t*(0.0003032));
    while(L0 > 360.0)
    {
        L0 -= 360.0;
    }
    while(L0 < 0.0)
    {
        L0 += 360.0;
    }
    return L0;		// in degrees
}

float calcGeomMeanAnomalySun(float t)
{
    float M = 357.52911 + t * (35999.05029 - 0.0001537 * t);
    return M;		// in degrees
}

float calcEccentricityEarthOrbit(float t)
{
    float e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
    return e;		// unitless
}

float calcSunEqOfCenter(float t)
{
    float m = calcGeomMeanAnomalySun(t);
    float mrad = degToRad(m);
    float sinm = sin(mrad);
    float sin2m = sin(mrad+mrad);
    float sin3m = sin(mrad+mrad+mrad);
    float C = sinm * (1.914602 - t * (0.004817 + 0.000014 * t)) + sin2m * (0.019993 - 0.000101 * t) + sin3m * 0.000289;
    return C;		// in degrees
}

float calcSunTrueLong(float t)
{
    float l0 = calcGeomMeanLongSun(t);
    float c = calcSunEqOfCenter(t);
    float O = l0 + c;
    return O;		// in degrees
}

float calcSunTrueAnomaly(float t)
{
    float m = calcGeomMeanAnomalySun(t);
    float c = calcSunEqOfCenter(t);
    float v = m + c;
    return v;		// in degrees
}

float calcSunRadVector(float t)
{
    float v = calcSunTrueAnomaly(t);
    float e = calcEccentricityEarthOrbit(t);
    float R = (1.000001018 * (1 - e * e)) / (1 + e * cos(degToRad(v)));
    return R;		// in AUs
}

float calcSunApparentLong(float t)
{
    float o = calcSunTrueLong(t);
    float omega = 125.04 - 1934.136 * t;
    float lambda = o - 0.00569 - 0.00478 * sin(degToRad(omega));
    return lambda;		// in degrees
}

float calcMeanObliquityOfEcliptic(float t)
{
    float seconds = 21.448 - t*(46.8150 + t*(0.00059 - t*(0.001813)));
    float e0 = 23.0 + (26.0 + (seconds/60.0))/60.0;
    return e0;		// in degrees
}

float calcObliquityCorrection(float t)
{
    float e0 = calcMeanObliquityOfEcliptic(t);
    float omega = 125.04 - 1934.136 * t;
    float e = e0 + 0.00256 * cos(degToRad(omega));
    return e;		// in degrees
}

float calcSunRtAscension(float t)
{
    float e = calcObliquityCorrection(t);
    float lambda = calcSunApparentLong(t);
    float tananum = (cos(degToRad(e)) * sin(degToRad(lambda)));
    float tanadenom = (cos(degToRad(lambda)));
    float alpha = radToDeg(atan2(tananum, tanadenom));
    return alpha;		// in degrees
}

float calcSunDeclination(float t)
{
    float e = calcObliquityCorrection(t);
    float lambda = calcSunApparentLong(t);

    float sint = sin(degToRad(e)) * sin(degToRad(lambda));
    float theta = radToDeg(asin(sint));
    return theta;		// in degrees
}

float calcEquationOfTime(float t)
{
    float epsilon = calcObliquityCorrection(t);
    float l0 = calcGeomMeanLongSun(t);
    float e = calcEccentricityEarthOrbit(t);
    float m = calcGeomMeanAnomalySun(t);

    float y = tan(degToRad(epsilon)/2.0);
    y *= y;

    float sin2l0 = sin(2.0 * degToRad(l0));
    float sinm   = sin(degToRad(m));
    float cos2l0 = cos(2.0 * degToRad(l0));
    float sin4l0 = sin(4.0 * degToRad(l0));
    float sin2m  = sin(2.0 * degToRad(m));

    float Etime = y * sin2l0 - 2.0 * e * sinm + 4.0 * e * y * sinm * cos2l0 - 0.5 * y * y * sin4l0 - 1.25 * e * e * sin2m;
    return radToDeg(Etime)*4.0;	// in minutes of time
}

/*
float calcHourAngleSunrise(float lat, float solarDec)
{
    float latRad = degToRad(lat);
    float sdRad  = degToRad(solarDec);
    float HAarg = (cos(degToRad(90.833))/(cos(latRad)*cos(sdRad))-tan(latRad) * tan(sdRad));
    float HA = acos(HAarg);
    return HA;		// in radians (for sunset, use -HA)
}

bool isNumber(char *inputVal)
{
    bool oneDecimal = false;
    for (size_t i = 0; i < strlen(inputVal); i++)
    {
        char oneChar = inputVal[i];
        if (i == 0 && (oneChar == '-' || oneChar == '+'))
        {
            continue;
        }
        if (oneChar == '.' && !oneDecimal)
        {
            oneDecimal = true;
            continue;
        }
        if (oneChar < '0' || oneChar > '9')
        {
            return false;
        }
    }
    return true;
}

float zeroPad(int n, digits) {
    n = n.toString();
    while (n.length < digits) {
        n = '0' + n;
    }
    return n;
}

*/

float getJD(struct tm *dt)
{
    float docmonth = dt->tm_mon + 1;
    float docday =   dt->tm_mday;
    float docyear =  dt->tm_year + 1900;
    float A = floor(docyear/100);
    float B = 2 - A + floor(A/4);
    float JD = floor(365.25*(docyear + 4716)) + floor(30.6001*(docmonth+1)) + docday + B - 1524.5;
    return JD;
}

float getTimeLocal(struct tm *dt)
{
    float dochr = dt->tm_hour;
    float docmn = dt->tm_min;
    float docsc = dt->tm_sec;
    bool docdst = dt->tm_isdst;
    if (docdst) {
        dochr -= 1;
    }
    float mins = dochr * 60 + docmn + docsc/60.0;
    return mins;
}

float calcAzEl(float T, float localtime, float latitude, float longitude, float zone)
{
    float eqTime = calcEquationOfTime(T);
    float theta  = calcSunDeclination(T);
    float azimuth = -1;
    float solarTimeFix = eqTime + 4.0 * longitude - 60.0 * zone;
    //float earthRadVec = calcSunRadVector(T);
    float trueSolarTime = localtime + solarTimeFix;
    while (trueSolarTime > 1440)
    {
        trueSolarTime -= 1440;
    }
    float hourAngle = trueSolarTime / 4.0 - 180.0;
    if (hourAngle < -180)
    {
        hourAngle += 360.0;
    }
    float haRad = degToRad(hourAngle);
    float csz = sin(degToRad(latitude)) * sin(degToRad(theta)) + cos(degToRad(latitude)) * cos(degToRad(theta)) * cos(haRad);
    if (csz > 1.0)
    {
        csz = 1.0;
    } else if (csz < -1.0)
    {
        csz = -1.0;
    }
    float zenith = radToDeg(acos(csz));
    float azDenom = ( cos(degToRad(latitude)) * sin(degToRad(zenith)) );
    float azRad = 0;
    if (fabs(azDenom) > 0.001) {
        azRad = (( sin(degToRad(latitude)) * cos(degToRad(zenith)) ) - sin(degToRad(theta))) / azDenom;
        if (fabs(azRad) > 1.0) {
            if (azRad < 0) {
                azRad = -1.0;
            } else {
                azRad = 1.0;
            }
        }
        azimuth = 180.0 - radToDeg(acos(azRad));
        if (hourAngle > 0.0) {
            azimuth = -azimuth;
        }
    } else {
        if (latitude > 0.0) {
            azimuth = 180.0;
        } else {
            azimuth = 0.0;
        }
    }
    if (azimuth < 0.0) {
        azimuth += 360.0;
    }
    float exoatmElevation = 90.0 - zenith;

    // Atmospheric Refraction correction

    float refractionCorrection = 0.0;
    if (exoatmElevation > 85.0) {
        refractionCorrection = 0.0;
    } else {
        float te = tan (degToRad(exoatmElevation));
        if (exoatmElevation > 5.0) {
            refractionCorrection = 58.1 / te - 0.07 / (te*te*te) + 0.000086 / (te*te*te*te*te);
        } else if (exoatmElevation > -0.575) {
            refractionCorrection = 1735.0 + exoatmElevation * (-518.2 + exoatmElevation * (103.4 + exoatmElevation * (-12.79 + exoatmElevation * 0.711) ) );
        } else {
            refractionCorrection = -20.774 / te;
        }
        refractionCorrection = refractionCorrection / 3600.0;
    }

    float solarZen = zenith - refractionCorrection;

    //printf("eqTime=%f, theta=%f, solarTimeFix=%f, earthRadVec=%f, trueSolarTime=%f, hourAngle=%f, haRad=%f, csz=%f, zenith=%f, azDenom=%f, azRad=%f, azimuth=%f, exoatmElevation=%f, refractionCorrection=%f, solarZ=%f\n",
    //        eqTime, theta, solarTimeFix, earthRadVec, trueSolarTime, hourAngle, haRad, csz, zenith, azDenom, azRad, azimuth, exoatmElevation, refractionCorrection, solarZen);

    if (solarZen > 108.0) {
        //its now officially dark
        azimuth = -1;
    }
    return (azimuth);
}

#if 0
float calcSolNoon(float jd, float longitude, float timezone, float dst)
{
    float tnoon = calcTimeJulianCent(jd - longitude/360.0);
    float eqTime = calcEquationOfTime(tnoon);
    float solNoonOffset = 720.0 - (longitude * 4) - eqTime; // in minutes
    float newt = calcTimeJulianCent(jd + solNoonOffset/1440.0);
    eqTime = calcEquationOfTime(newt);
    float solNoonLocal = 720 - (longitude * 4) - eqTime + (timezone*60.0);// in minutes
    if(dst) solNoonLocal += 60.0;
    while (solNoonLocal < 0.0) {
        solNoonLocal += 1440.0;
    }
    while (solNoonLocal >= 1440.0) {
        solNoonLocal -= 1440.0;
    }
    return solNoonLocal;
}
/*
float dayString(jd, next, flag)
{
    // returns a string in the form DDMMMYYYY[ next] to display prev/next rise/set
    // flag=2 for DD MMM, 3 for DD MM YYYY, 4 for DDMMYYYY next/prev
    if ( (jd < 900000) || (jd > 2817000) ) {
        float output = "error"
    } else {
        float z = floor(jd + 0.5);
        float f = (jd + 0.5) - z;
        if (z < 2299161) {
            float A = z;
        } else {
            alpha = floor((z - 1867216.25)/36524.25);
            float A = z + 1 + alpha - floor(alpha/4);
        }
        float B = A + 1524;
        float C = floor((B - 122.1)/365.25);
        float D = floor(365.25 * C);
        float E = floor((B - D)/30.6001);
        float day = B - D - floor(30.6001 * E) + f;
        float month = (E < 14) ? E - 1 : E - 13;
        float year = ((month > 2) ? C - 4716 : C - 4715);
        if (flag == 2)
            float output = zeroPad(day,2) + " " + monthList[month-1].abbr;
        if (flag == 3)
            float output = zeroPad(day,2) + monthList[month-1].abbr + year.toString();
        if (flag == 4)
            float output = zeroPad(day,2) + monthList[month-1].abbr + year.toString() + ((next) ? " next" : " prev");
    }
    return output;
}
*/
float calcSunriseSetUTC(int rise, float JD, float latitude, float longitude)
{
    float t = calcTimeJulianCent(JD);
    float eqTime = calcEquationOfTime(t);
    float solarDec = calcSunDeclination(t);
    float hourAngle = calcHourAngleSunrise(latitude, solarDec);
    //alert("HA = " + radToDeg(hourAngle));
    if (!rise) hourAngle = -hourAngle;
    float delta = longitude + radToDeg(hourAngle);
    float timeUTC = 720 - (4.0 * delta) - eqTime;	// in minutes
    return timeUTC;
}

float calcJDofNextPrevRiseSet(int next, int rise, float JD, float latitude, float longitude, float tz, float dst)
{
    float julianday = JD;
    float increment = ((next) ? 1.0 : -1.0);

    float time = calcSunriseSetUTC(rise, julianday, latitude, longitude);
    //while(!isNumber(time)){
    //    julianday += increment;
    //    time = calcSunriseSetUTC(rise, julianday, latitude, longitude);
    //}
    float timeLocal = time + tz * 60.0 + ((dst) ? 60.0 : 0.0);
    while ((timeLocal < 0.0) || (timeLocal >= 1440.0))
    {
        float incr = ((timeLocal < 0) ? 1 : -1);
        timeLocal += (incr * 1440.0);
        julianday -= incr;
    }
    return julianday;
}


// rise = 1 for sunrise, 0 for sunset
float calcSunriseSet(int rise, float JD, float latitude, float longitude, float timezone, float dst)
{
    //float id = ((rise) ? "risebox" : "setbox");
    float timeUTC = calcSunriseSetUTC(rise, JD, latitude, longitude);
    float newTimeUTC = calcSunriseSetUTC(rise, JD + timeUTC/1440.0, latitude, longitude);
    //if (isNumber(newTimeUTC))
    if (true)
    {
        float timeLocal = newTimeUTC + (timezone * 60.0);
        if (false) {
            float riseT = calcTimeJulianCent(JD + newTimeUTC/1440.0);
            float riseAz = calcAzEl(1, riseT, timeLocal, latitude, longitude, timezone);
            if (rise) {
                //showLineGeodesic2("sunrise", "#00aa00", riseAz);
            } else {
                //showLineGeodesic2("sunset", "#ff0000", riseAz);
            }
        }
        timeLocal += ((dst) ? 60.0 : 0.0);
        if ( (timeLocal >= 0.0) && (timeLocal < 1440.0) ) {
            //document.getElementById(id).value = timeString(timeLocal,2)
        } else  {
            float jday = JD;
            float increment = ((timeLocal < 0) ? 1 : -1);
            while ((timeLocal < 0.0)||(timeLocal >= 1440.0)) {
                timeLocal += increment * 1440.0;
                jday -= increment;
            }
            //document.getElementById(id).value = timeDateString(jday,timeLocal)
        }
    } else { // no sunrise/set found
        /*
        float jdy;
        float doy = calcDoyFromJD(JD);
        if ( ((latitude > 66.4) && (doy > 79) && (doy < 267)) ||
                ((latitude < -66.4) && ((doy < 83) || (doy > 263))) )
        {   //previous sunrise/next sunset
            if (rise) { // find previous sunrise
                jdy = calcJDofNextPrevRiseSet(0, rise, JD, latitude, longitude, timezone, dst);
            } else { // find next sunset
                jdy = calcJDofNextPrevRiseSet(1, rise, JD, latitude, longitude, timezone, dst);
            }
            //document.getElementById(((rise)? "risebox":"setbox")).value = dayString(jdy,0,3);
        } else {   //previous sunset/next sunrise
            if (rise == 1) { // find previous sunrise
                jdy = calcJDofNextPrevRiseSet(1, rise, JD, latitude, longitude, timezone, dst);
            } else { // find next sunset
                jdy = calcJDofNextPrevRiseSet(0, rise, JD, latitude, longitude, timezone, dst);
            }
            //document.getElementById(((rise)? "risebox":"setbox")).value = dayString(jdy,0,3);
        }
        */
    }
    return 0;
}
#endif

float calculate(float lat, float lng, struct tm *dt) {
    float jday = getJD(dt);
    float tl = getTimeLocal(dt);
    float tz = dt->tm_gmtoff/3600;
    //bool  dst = dt->tm_isdst;
    float total = jday + tl/1440.0 - tz/24.0;
    float T = calcTimeJulianCent(total);
    float azi = calcAzEl(T, tl, lat, lng, tz);
    //calcSolNoon(jday, lng, tz, dst)
    //float rise = calcSunriseSet(1, jday, lat, lng, tz, dst);
    //float set  = calcSunriseSet(0, jday, lat, lng, tz, dst);
    return azi;
}

float calcSun(float lat, float lon) {

    struct timeval now;
    //char timestr[64];

    gettimeofday(&now, NULL);
    struct tm* now_local = localtime( &now.tv_sec );
    //strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", now_local);
    float azi = calculate(lat/10000000, lon/10000000, now_local);
    //printf("%s azi=%f\n", timestr, azi);
    return azi;
}
