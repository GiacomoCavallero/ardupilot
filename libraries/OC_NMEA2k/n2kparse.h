#ifndef __N2KPARSE_H
#define __N2KPARSE_H

#ifndef WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

#include <stdint.h>
#include <string.h>
#include "pgn.h"
//////////////////////////////////////////////////////////////////////////////////////
//NGT specific
extern const unsigned char NGT_STARTUP_SEQ[3];

/*
* Defines to interface with an Actisense NGT-1
*/

/* ASCII characters used to mark packet start/stop */

#define STX (0x02)  /* Start packet */
#define ETX (0x03)  /* End packet */
#define DLE (0x10)  /* Start pto encode a STX or ETX send DLE+STX or DLE+ETX */
#define ESC (0x1B)  /* Escape */

/* Actisense message structure is:
DLE STX <command> <len> [<data> ...]  <checksum> DLE ETX
<command> is a byte from the list below.
In <data> any DLE characters are double escaped (DLE DLE).
<len> encodes the unescaped length.
<checksum> is such that the sum of all unescaped data bytes plus the command
byte plus the length adds up to zero, modulo 256.
*/

#define N2K_MSG_RECEIVED (0x93)  /* Receive standard N2K message */
#define N2K_MSG_SEND     (0x94)  /* Send N2K message */
#define NGT_MSG_RECEIVED (0xA0)  /* Receive NGT specific message */
#define NGT_MSG_SEND     (0xA1)  /* Send NGT message */


int messageReceived(const unsigned char * msg, int msgLen, int &command, int &payLen);
int readNGT1Byte(unsigned char c, unsigned char *msg);

enum ValType {
	ValType_Unknown = -1,
	ValType_Integer,
	ValType_Double,
	ValType_Lookup,
	ValType_Date,
	ValType_Time,
	ValType_ASCII
};

struct FieldValue
{
	ValType type;
	char *name;
	double resolution;
	int precision;
	double dVal;
	int64_t val;
	char data[32];
	char units[32];
	char lookup[64];
	bool valid;
	FieldValue()
		: type(ValType_Unknown)
		, name(0)
		, resolution(0)
		, precision(0)
		, dVal(0.0)
		, val(0)
		, valid(true)
	{
		*data = 0;
		*units = 0;
		*lookup = 0;
	}
};

struct MsgVals
{
	int count;
	unsigned int pgn;
	unsigned int src;
	unsigned int dst;
	char *desc;
	FieldValue *pVals;
	MsgVals(int count_)
		: count(count_)
		, pgn(0)
		, src(0)
		, dst(0)
		, desc(0)
		, pVals(new FieldValue[count])
	{
	}
	~MsgVals()
	{
		if (pVals)
			delete[] pVals;
	}

	bool isValid(const char *name)
	{
		int i;
		for (i = 0; i < count; i++)
			if (!strcmp(pVals[i].name, name))
				return pVals[i].valid;
		return false;
	}
	char *getLookup(const char *name)
	{
		int i;
		for (i = 0; i < count; i++)
			if (!strcmp(pVals[i].name, name))
				return pVals[i].lookup;
		return (char *)"";
	}
	double getDouble(const char *name)
	{
		int i;
		for (i = 0; i < count; i++)
			if (!strcmp(pVals[i].name, name))
				return pVals[i].dVal;
		return 0.0;
	}
	int getInteger(const char *name)
	{
		int i;
		for (i = 0; i < count; i++)
			if (!strcmp(pVals[i].name, name))
				return (int)pVals[i].val;
		return 0;
	}
	const char *getDateOrTime(const char *name)
	{
		int i;
		for (i = 0; i < count; i++)
			if (!strcmp(pVals[i].name, name))
				return pVals[i].data;
		return 0;
	}
	const char *getUnits(const char *name)
	{
		int i;
		for (i = 0; i < count; i++)
			if (!strcmp(pVals[i].name, name))
				return pVals[i].units;
		return 0;
	}
};

unsigned int n2kMessageReceived(const unsigned char * msg, int msgLen, MsgVals *&pmv);

#ifndef WIN32
#pragma GCC diagnostic pop
#endif

#endif
