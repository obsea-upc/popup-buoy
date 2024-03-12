// -------------------------------------------------------------------------- //
//! @file    previpass_util.c
//! @brief   Satellite pass prediction utils functions
//! @author  Kineis
//! @date    2020-01-14
// -------------------------------------------------------------------------- //

// -------------------------------------------------------------------------- //
// Includes
// -------------------------------------------------------------------------- //

#include "previpass_util.h"

#include <math.h>


// -------------------------------------------------------------------------- //
//! @addtogroup ARGOS-PASS-PREDICTION-LIBS
//! @{
// -------------------------------------------------------------------------- //


// -------------------------------------------------------------------------- //
//! Number of days before each month:
//!    - ek_quant( 1...12, 1 ) for leap years
//!    - ek_quant( 1...12, 2 ) for non leap years
// -------------------------------------------------------------------------- //

static
const uint32_t
__ek_quanti[12][2] = {

	{   0,   0 },
	{  31,  31 },
	{  60,  59 },
	{  91,  90 },
	{ 121, 120 },
	{ 152, 151 },
	{ 182, 181 },
	{ 213, 212 },
	{ 244, 243 },
	{ 274, 273 },
	{ 305, 304 },
	{ 335, 334 }
};


// -------------------------------------------------------------------------- //
// PREVIPASS_UTIL_sat_elevation_distance2
// -------------------------------------------------------------------------- //

float
PREVIPASS_UTIL_sat_elevation_distance2(
	float elevationDeg,
	float semiMajorAxisKm
)
{
	float tmp;

	// Conversion in radians
	tmp = elevationDeg * C_MATH_DEG_TO_RAD;

	// Compute required angular distance
	tmp = C_MATH_HALF_PI - tmp - asinf(C_MATH_EARTH_RADIUS / semiMajorAxisKm * cosf(tmp));

	// From angular distance to cartesian distance
	tmp = 2.0f * sinf(tmp / 2.0f);

	// Squared distance (avoid sqrt for future comparisions)
	return tmp * tmp;
}


// -------------------------------------------------------------------------- //
// su_distance
// -------------------------------------------------------------------------- //

float
PREVIPASS_UTIL_sat_point_distance2(
	uint32_t secondsSinceBulletin,
	float    xBeaconCartesian,
	float    yBeaconCartesian,
	float    zBeaconCartesian,
	float    mean_motion,
	float    sin_inclination,
	float    cos_inclination,
	float    ascNodeRad,
	float    earthRevPerSec
)
{
	float lat_sat ;        // Latitude of the satellite
	float long_sat_an ;    // Longitude of the satellite without earth rotation
	float long_sat ;       // Longitude of the satellite
	float x_sat_earthRef ; // Satellite cartesian position
	float y_sat_earthRef ; // Satellite cartesian position
	float z_sat_earthRef ; // Satellite cartesian position

	// Calculation of the satellite latitude
	lat_sat = asinf(sinf(mean_motion * (uint32_t)secondsSinceBulletin) * sin_inclination);

	// Calculation of the satellite longitude
	long_sat_an = atanf(tanf(mean_motion * (uint32_t)secondsSinceBulletin) * cos_inclination);
	if (cosf(mean_motion * (uint32_t)secondsSinceBulletin) < 0)
		long_sat_an += C_MATH_PI;


	// Earth rotation
	long_sat = ascNodeRad + long_sat_an + earthRevPerSec * (uint32_t)secondsSinceBulletin;
	long_sat = fmodf(long_sat, C_MATH_TWO_PI);
	if (long_sat < 0)
		long_sat += C_MATH_TWO_PI;


	// Spheric satellite positions calculation in TR
	x_sat_earthRef = cosf(lat_sat) * cosf(long_sat);
	y_sat_earthRef = cosf(lat_sat) * sinf(long_sat);
	z_sat_earthRef = sinf(lat_sat);

	// Calculation of the distance between the point and the satellite
	return (x_sat_earthRef - xBeaconCartesian) * (x_sat_earthRef - xBeaconCartesian) +
		(y_sat_earthRef - yBeaconCartesian) * (y_sat_earthRef - yBeaconCartesian) +
		(z_sat_earthRef - zBeaconCartesian) * (z_sat_earthRef - zBeaconCartesian);
}


// -------------------------------------------------------------------------- //
// PREVIPASS_UTIL_date_calendar_stu90
// -------------------------------------------------------------------------- //

void
PREVIPASS_UTIL_date_calendar_stu90(
	struct CalendarDateTime_t  dateTime,
	uint32_t           *sec90
)
{
	// Number of years since 1990
	uint32_t numberOfYears = (uint32_t)dateTime.gpsYear - 1990;

	// Number of leap years since 1990
	uint32_t numberOfLeapYears = ((uint32_t)dateTime.gpsYear - 1 - 1900) / 4 - 22;

	// Leap year flag (0: leap, 1: non leap)
	uint8_t isLeapYear = MIN(dateTime.gpsYear % 4, 1);

	// Number of days since 1990
	uint32_t numberOfDays = (uint32_t)numberOfYears * 365 + numberOfLeapYears
		+ __ek_quanti[dateTime.gpsMonth-1][isLeapYear]
		+ dateTime.gpsDay - 1;

	// Conversion in seconds
	*sec90 = (uint32_t)numberOfDays     * 86400
		+ (uint32_t)dateTime.gpsHour   *  3600
		+ (uint32_t)dateTime.gpsMinute *    60
		+ (uint32_t)dateTime.gpsSecond;
}


// -------------------------------------------------------------------------- //
// PREVIPASS_UTIL_date_stu90_calendar
// -------------------------------------------------------------------------- //

void
PREVIPASS_UTIL_date_stu90_calendar(
	uint32_t            sec90,
	struct CalendarDateTime_t *dateTime
)
{
	// Number of days since 1990
	uint32_t numberOfDays = (uint32_t)sec90 / 86400;

	// Hours, minutes, seconds
	uint32_t temp = (uint32_t)sec90 - ((uint32_t)numberOfDays * 86400);

	dateTime->gpsHour = (uint8_t)((uint32_t)temp / 3600);
	temp -= (uint32_t)dateTime->gpsHour * 3600;
	dateTime->gpsMinute = (uint8_t)((uint32_t)temp / 60);
	temp -= (uint32_t)dateTime->gpsMinute * 60;
	dateTime->gpsSecond = (uint8_t)(temp);

	// Number of years since 1990
	uint16_t numberOfYears = (uint16_t)(((uint32_t)numberOfDays + 0.5f) / 365.25f);

	dateTime->gpsYear = (uint16_t)(numberOfYears + 1990);

	// Number of leap years since 1990
	uint8_t numberOfLeapYears = (uint8_t)((dateTime->gpsYear - 1 - 1900) / 4 - 22);

	// Leap year flag (0: leap, 1: non leap)
	uint8_t isLeapYear = MIN(dateTime->gpsYear % 4, 1);

	// Number of day since start of year
	numberOfDays -= (uint32_t)numberOfYears * 365 + numberOfLeapYears - 1;

	// Month number in year
	dateTime->gpsMonth = 1;
	while ((numberOfDays > __ek_quanti[dateTime->gpsMonth - 1][isLeapYear])
			&& (dateTime->gpsMonth <= 12))
		++(dateTime->gpsMonth);
	--(dateTime->gpsMonth);

	// Day number in month
	dateTime->gpsDay = (uint8_t)((uint32_t)numberOfDays -  __ek_quanti[dateTime->gpsMonth - 1][isLeapYear]);
}


// -------------------------------------------------------------------------- //
//! @} (end addtogroup ARGOS-PASS-PREDICTION-LIBS)
// -------------------------------------------------------------------------- //
