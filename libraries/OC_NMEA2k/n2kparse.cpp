//n2kparse.cpp
//lots of code from canboat
//
#include "n2kparse.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

#ifndef min
#define min(a,b) (a < b ? a : b)
#endif

/**
* Handle a byte coming in from the NGT1.
*
*/
enum MSG_State
{
	MSG_START,
	MSG_ESCAPE,
	MSG_MESSAGE
};

#define DATE_LENGTH 60

#if 0
#ifndef WIN32
const char * now(char str[DATE_LENGTH])
{
	struct timeval tv;
	time_t t;
	struct tm tm;
	int msec;
	size_t len;

	if (gettimeofday(&tv, (void *)0) == 0)
	{
		t = tv.tv_sec;
		msec = tv.tv_usec / 1000L;
		gmtime_r(&t, &tm);
		strftime(str, DATE_LENGTH - 5, "%Y-%m-%dT%H:%M:%S", &tm);
		len = strlen(str);
		snprintf(str + len, DATE_LENGTH - len, ".%3.3dZ", msec);
	}
	else
	{
		strcpy(str, "?");
	}

	return (const char *)str;
}

#else
const char * now(char str[DATE_LENGTH])
{
	SYSTEMTIME gt;
	GetSystemTime(&gt);
	sprintf(str, "%04d-%02d-%02dT%02d:%02d:%02d:%03dZ", gt.wYear, gt.wMonth, gt.wDay, gt.wHour, gt.wMinute, gt.wSecond, gt.wMilliseconds);
	/*
	struct _timeb timebuffer;
	struct tm tm;
	size_t len;

	_ftime_s(&timebuffer);
	gmtime_s(&tm, &timebuffer.time);
	strftime(str, DATE_LENGTH - 5, "%Y-%m-%dT%H:%M:%S", &tm);
	len = strlen(str);
	snprintf(str + len, DATE_LENGTH - len, ".%3.3dZ", timebuffer.millitm);
	*/

	return (const char *)str;
}
#endif
#endif

void ngtMessageReceived(const unsigned char * msg, size_t msgLen)
{
#if 0
	size_t i;
	char line[1000];
	char * p;
	if (msgLen < 12)
	{
		//Log4_ERROR("Ignore short msg len = %zu", msgLen);
		return;
	}

	//sprintf(line, "%s,%u,%u,%u,%u,%u", now(dateStr), 0, 0x40000 + msg[0], 0, 0, (unsigned int)msgLen - 1);
	//p = line + strlen(line);
	for (i = 1; i < msgLen && p < line + sizeof(line) - 5; i++)
	{
		//sprintf(p, ",%02x", msg[i]);
		p += 3;
	}
	*p++ = 0;

	//puts(line);
	//fflush(stdout);
#endif
}

static bool pgn_init = false;

void InitPGN()
{
	if (pgn_init)
		return;
	fillManufacturers();
	fillFieldCounts();
	checkPgnList();
	pgn_init = true;
}

unsigned int n2kMessageReceived(const unsigned char * msg, int msgLen, MsgVals *&pmv)
{
	InitPGN();

	unsigned int src, dst;
	unsigned int _pgn;
	unsigned int len;

	if (msgLen < 11)
	{
		//Log4_ERROR("Ignoring N2K message - too short");
		return 0;
	}
	//prio = msg[0];
	_pgn = (unsigned int)msg[1] + 256 * ((unsigned int)msg[2] + 256 * (unsigned int)msg[3]);
	dst = msg[4];
	src = msg[5];
	/* Skip the timestamp logged by the NGT-1-A in bytes 6-9 */
	len = msg[10];

	if (len > 223)
	{
		//Log4_ERROR("Ignoring N2K message - too long (%u)", len);
		return 0;
	}

	if (true)
	{
		uint8_t ds = 0;
		Pgn *pgn = getMatchingPgn(_pgn, &ds, len);
		uint8_t *dataStart = (uint8_t *)&msg[11];
		if (pgn)
		{
			pmv = new MsgVals(pgn->fieldCount);

			int startBit = 0;
			uint8_t *data = dataStart;
			//bool matchedFixedField = true;
			//bool hasFixedField = false;

			uint32_t i;
			// Iterate over fields
			for (i = 0, startBit = 0, data = dataStart; i < pgn->fieldCount; i++)
			{
				const Field *field = &pgn->fieldList[i];
				int bits = field->size;
				pmv->pgn = _pgn;
				pmv->desc = pgn->description;
				pmv->pVals[i].name = field->name;
				pmv->pVals[i].resolution = field->resolution;
				pmv->src = src;
				pmv->dst = dst;
				if (field->units && field->units[0] == '=')
				{
#if 0
					int64_t value, desiredValue;
					int64_t maxValue;

					hasFixedField = true;
					extractNumber(field, data, startBit, field->size, &value, &maxValue);
					desiredValue = strtol(field->units + 1, 0, 10);
					if (value != desiredValue)
					{
						matchedFixedField = false;
						break;
					}
#endif
				}
				else
				{
					int64_t value;
					int64_t maxValue;

					extractNumber(field, data, startBit, field->size, &value, &maxValue);
					if (value != maxValue)
					{
						if (field->resolution == RES_LOOKUP && field->units)
						{
							char lookfor[20];
							char * s, *e;

							sprintf(lookfor, ",%lld=", value);
							s = strstr(field->units, lookfor);
							if (s)
							{
								s += strlen(lookfor);
								e = strchr(s, ',');
								e = e ? e : s + strlen(s);
								char buf[64];
								sprintf(buf, "%.*s",(int)(e - s), s);
								pmv->pVals[i].type = ValType_Lookup;
								pmv->pVals[i].val = value;
								strncpy(pmv->pVals[i].lookup, buf, sizeof(pmv->pVals[i].lookup));
							}
						}
						else if (field->resolution == RES_LATITUDE || field->resolution == RES_LONGITUDE)
						{
							//uint64_t absVal;
							int64_t value_;
							int bits_ = field->size;
							int bytes = (bits_ + 7) / 8;
							uint8_t * dataEnd = dataStart + len;
							bytes = min(bytes, (dataEnd - data));

							value_ = 0;
							memcpy(&value_, data, bytes);
							if (bytes == 4 && ((data[3] & 0x80) > 0))
							{
								value_ |= UINT64_C(0xffffffff00000000);
							}
							if (value_ > ((bytes == 8) ? INT64_C(0x7ffffffffffffffd) : INT64_C(0x7ffffffd)))
							{
								pmv->pVals[i].valid = false;
							}

							if (bytes == 8)
							{
								value_ /= INT64_C(1000000000);
							}
							//absVal = (value_ < 0) ? -value_ : value_;

							double dd = 0;
							if (pmv->pVals[i].valid)
								dd = (double)value_ / (double)RES_LAT_LONG_PRECISION;

							pmv->pVals[i].type = ValType_Double;
							pmv->pVals[i].dVal = dd;
							pmv->pVals[i].precision = 7;
							strncpy(pmv->pVals[i].units, "", sizeof(pmv->pVals[i].units));
						}
						else if (field->resolution == RES_DATE)
						{
							char buf[sizeof("2008.03.10") + 1];
							time_t t;
							struct tm * tm;

							if (value >= 0xfffd)
							{
								pmv->pVals[i].valid = false;
							}

							t = value * 86400;
							tm = gmtime(&t);
							if (!tm)
							{
								pmv->pVals[i].valid = false;
								//logAbort("Unable to convert %u to gmtime\n", (unsigned int) t);
								//break;
							}

							if (pmv->pVals[i].valid)
								strftime(buf, sizeof(buf), "%Y.%m.%d", tm);
							else
								buf[0] = 0;

							pmv->pVals[i].type = ValType_Date;
							strncpy(pmv->pVals[i].data, buf, sizeof(pmv->pVals[i].data));
						}
						else if (field->resolution == RES_TIME)
						{
							uint32_t hours;
							uint32_t minutes;
							uint32_t seconds;
							uint32_t units;
							const uint32_t unitspersecond = 10000;

							if (value >= 0xfffffffd)
							{
								pmv->pVals[i].valid = false;
								//break;
							}

							seconds = (uint32_t)(value / unitspersecond);
							units = (uint32_t)(value % unitspersecond);
							minutes = seconds / 60;
							seconds = seconds % 60;
							hours = minutes / 60;
							minutes = minutes % 60;

							pmv->pVals[i].type = ValType_Time;
							char buf[64];
							if (units)
								sprintf(buf, "%02u:%02u:%02u.%05u", hours, minutes, seconds, units);
							else
								sprintf(buf, "%02u:%02u:%02u", hours, minutes, seconds);
							if (!pmv->pVals[i].valid)
								buf[0] = 0;
							strncpy(pmv->pVals[i].data, buf, sizeof(pmv->pVals[i].data));
						}
						else if (field->resolution == RES_PRESSURE)
						{
							int32_t pressure;
							double bar;
							//double psi;

							if (value >= 0xfffd)
							{
								pmv->pVals[i].valid = false;
								//break;
							}

							// There are three types of known pressure: unsigned hectopascal, signed kpa, unsigned kpa.

							if (field->hasSign)
							{
								pressure = (int16_t)value;
							}
							else
							{
								pressure = (uint32_t)value;
							}
							// Now scale pascal properly, it is in hPa or kPa.
							if (field->units)
							{
								switch (field->units[0])
								{
								case 'h':
								case 'H':
									pressure *= 100;
									break;
								case 'k':
								case 'K':
									pressure *= 1000;
								}
							}

							if (pmv->pVals[i].valid)
								bar = pressure / 100000.0; /* 1000 hectopascal = 1 Bar */
							else
								bar = 0;
							//psi = pressure / 1450.377; /* Silly but still used in some parts of the world */

							pmv->pVals[i].type = ValType_Double;
							pmv->pVals[i].dVal = bar;
							pmv->pVals[i].precision = 3;
							strncpy(pmv->pVals[i].units, "bar", sizeof(pmv->pVals[i].units));
						}
						else if (field->resolution == RES_TEMPERATURE)
						{
							if (value >= 0xfffd)
							{
								pmv->pVals[i].valid = false;
								//break;
							}
							double k = value / 100.0;
							double c = k - 273.15;
							//double f = c * 1.8 + 32;

							pmv->pVals[i].type = ValType_Double;
							pmv->pVals[i].dVal = pmv->pVals[i].valid ? c : 0;
							strncpy(pmv->pVals[i].units, "C", sizeof(pmv->pVals[i].units));
						}
						else if (field->resolution == RES_INTEGER)
						{
							pmv->pVals[i].type = ValType_Integer;
							pmv->pVals[i].val = value;
						}
						else if (field->resolution > 0.0 && field->resolution < 1.0)
						{
							int precision;
							double r;
							const char * units = field->units;

							double a = (double)value * field->resolution;

							precision = 0;
							for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
							{
								precision++;
							}

							if (field->resolution == RES_RADIANS)
							{
								units = "rad";
							}
							else if (field->resolution == RES_ROTATION || field->resolution == RES_HIRES_ROTATION)
							{
								units = "rad/s";
							}
							pmv->pVals[i].precision = precision;
							pmv->pVals[i].type = ValType_Double;
							pmv->pVals[i].dVal = a;
							strncpy(pmv->pVals[i].units, units ? units : "", sizeof(pmv->pVals[i].units));
						}
						else
						{
							pmv->pVals[i].type = ValType_Integer;
							pmv->pVals[i].val = value;
						}
					}
					else
					{
						pmv->pVals[i].val = 0;
					}
				}
				startBit += bits;
				data += startBit / 8;
				startBit %= 8;
			}
		}
	}

	return _pgn;
}

//return -1 on error
//return 0 for non N2K
//return 1 for N2K
//
int messageReceived(const unsigned char * msg, int msgLen, int &command, int &payLen)
{
	unsigned char checksum = 0;
	int i;

	if (msgLen < 3)
	{
		//Log4_ERROR("Ignore short command len = %zu", msgLen);
		return -1;
	}

	for (i = 0; i < msgLen; i++)
	{
		checksum += msg[i];
	}
	if (checksum)
	{
		//Log4_ERROR("Ignoring message with invalid checksum");
		return -1;
	}

	command = msg[0];
	payLen = msg[1];

	//Log4_DEBUG("message command = %02x len = %u", command, payloadLen);

	if (command == N2K_MSG_RECEIVED)
	{
		//MsgVals *pmv = 0;
		//n2kMessageReceived(msg + 2, payloadLen, pmv);
		//if (pmv)
		//{
		//	delete pmv;
		//}
		return 1;
	}
	else if (command == NGT_MSG_RECEIVED)
	{
		//ngtMessageReceived(msg + 2, payloadLen);
		return 0;
	}
	return -1;
}

bool isFile = false;
int readNGT1Byte(unsigned char c, unsigned char *msg)
{
	static enum MSG_State state = MSG_START;
	static bool startEscape = false;
	static bool noEscape = false;
	static unsigned char buf[500];
	static unsigned char * head = buf;

	//Log4_DEBUG("received byte %02x state=%d offset=%d", c, state, head - buf);

	if (state == MSG_START)
	{
		if ((c == ESC) && isFile)
		{
			noEscape = true;
		}
	}

	if (state == MSG_ESCAPE)
	{
		if (c == ETX)
		{
			//messageReceived(buf, head - buf);
			int len = head - buf;
			memcpy(msg, buf, len);
			head = buf;
			state = MSG_START;
			return len;
		}
		else if (c == STX)
		{
			head = buf;
			state = MSG_MESSAGE;
		}
		else if ((c == DLE) || ((c == ESC) && isFile) || noEscape)
		{
			*head++ = c;
			state = MSG_MESSAGE;
		}
		else
		{
			//Log4_ERROR("DLE followed by unexpected char %02X, ignore message", c);
			state = MSG_START;
		}
	}
	else if (state == MSG_MESSAGE)
	{
		if (c == DLE)
		{
			state = MSG_ESCAPE;
		}
		else if (isFile && (c == ESC) && !noEscape)
		{
			state = MSG_ESCAPE;
		}
		else
		{
			*head++ = c;
		}
	}
	else
	{
		if (c == DLE)
		{
			state = MSG_ESCAPE;
		}
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////

