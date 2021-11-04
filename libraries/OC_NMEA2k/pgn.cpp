
#define GLOBALS

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif
#include "pgn.h"

#ifdef WIN32
# include <stdio.h>
# include <windows.h>
#else
# include <stdint.h>
# include <stdio.h>
# include <stdbool.h>
# include <stdlib.h>
# include <stdio.h>
#  include <malloc.h>
# include <string.h>
# include <stdarg.h>
# include <inttypes.h>
# include <sys/time.h>
# include <ctype.h>
#endif

#ifndef min
#define min(a,b) (a < b ? a : b)
#endif

Pgn *searchForPgn(int pgn)
{
  int start = 0;
  int end = pgnListSize;
  int mid;

  while (start <= end)
  {
    mid = (start + end) / 2;
    if (pgn == (int)pgnList[mid].pgn)
    {
      while (mid && pgn == (int)pgnList[mid - 1].pgn)
      {
        mid--;
      }
      return &pgnList[mid];
    }
    if (pgn < (int)pgnList[mid].pgn)
    {
      end = mid - 1;
    }
    else
    {
      start = mid + 1;
    }
  }
  return 0;
}

/**
 * Return the last Pgn entry for which unknown == true && prn is smaller than requested.
 * This is slower, but is not used often.
 */
static Pgn *searchForUnknownPgn(int pgnId)
{
  Pgn *unknown = pgnList;
  Pgn *pgn;

  for (pgn = pgnList; pgn < pgnList + pgnListSize; pgn++)
  {
    if (pgn->unknownPgn)
    {
      unknown = pgn;
    }
    if ((int)pgn->pgn > pgnId)
    {
      break;
    }
  }
  return unknown;
}

Pgn* getMatchingPgn(int pgnId, uint8_t *dataStart, int length)
{
  Pgn *pgn = searchForPgn(pgnId);
  int prn;
  uint8_t i;

  if (!pgn)
  {
    pgn = searchForUnknownPgn(pgnId);
  }

  prn = pgn->pgn;

  if (pgn == pgnListEnd() - 1 || (int)pgn[1].pgn != prn)
  {
    // Don't bother complex search if there is only one PGN with this PRN.
    return pgn;
  }

  for (; (int)pgn->pgn == prn; pgn++) // we never get here for the last pgn, so no need to check for end of list
  {
    int startBit = 0;
    uint8_t *data = dataStart;

    bool matchedFixedField = true;
    bool hasFixedField = false;

    /* There is a next index that we can use as well. We do so if the 'fixed' fields don't match */

    if (!pgn->fieldCount)
    {
      //logError("Internal error: %p PGN %d offset %u '%s' has no fields\n", pgn, prn, (unsigned) (pgn - pgnList), pgn->description);
      for (i = 0; pgn->fieldList[i].name; i++)
      {
        //logInfo("Field %d: %s\n", i, pgn->fieldList[i].name);
      }
      // exit(2);
      pgn->fieldCount = i;
    }

    // Iterate over fields
    for (i = 0, startBit = 0, data = dataStart; i < pgn->fieldCount; i++)
    {
      const Field *field = &pgn->fieldList[i];
      int bits = field->size;

      if (field->units && field->units[0] == '=')
      {
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
      }
      startBit += bits;
      data += startBit / 8;
      startBit %= 8;
    }
    if (!hasFixedField)
    {
      //logDebug("Cant determine prn choice, return prn=%d variation '%s'\n", prn, pgn->description);
      return pgn;
    }
    if (matchedFixedField)
    {
      return pgn;
    }
  }
  return 0;
}

void checkPgnList(void)
{
  int i;
  int prn = 0;

  for (i = 0; i < (int)pgnListSize; i++)
  {
    Pgn * pgn;

    if ((int)pgnList[i].pgn < prn)
    {
      //logError("Internal error: PGN %d is not sorted correctly\n", pgnList[i].pgn);
      exit(2);
    }
    if ((int)pgnList[i].pgn == prn)
    {
      continue;
    }
    prn = pgnList[i].pgn;
    pgn = searchForPgn(prn);
    if (pgn != &pgnList[i])
    {
      //logError("Internal error: PGN %d is not found correctly\n", prn);
      exit(2);
    }
  }
}

Field * getField(uint32_t pgnId, uint32_t field)
{
  Pgn* pgn = searchForPgn(pgnId);
  if (pgn && field < pgn->fieldCount)
  {
    return pgn->fieldList + field;
  }
  return 0;
}

/*
 *
 * This is perhaps as good a place as any to explain how CAN messages are layed out by the
 * NMEA. Basically, it's a mess once the bytes are recomposed into bytes (the on-the-wire
 * format is fine).
 *
 * For fields that are aligned on bytes there isn't much of an issue, they appear in our
 * buffers in standard Intel 'least endian' format.
 * For instance the MMSI # 244050447 is, in hex: 0x0E8BEA0F. This will be found in the CAN data as:
 * byte x+0: 0x0F
 * byte x+1: 0xEA
 * byte x+2: 0x8B
 * byte x+3: 0x0e
 *
 * To gather together we loop over the bytes, and keep increasing the magnitude of what we are
 * adding:
 *    for (i = 0, magnitude = 0; i < 4; i++)
 *    {
 *      value += data[i] << magnitude;
 *      magnitude += 8;
 *    }
 *
 * However, when there are two bit fields after each other, lets say A of 2 and then B of 6 bits:
 * then that is layed out MSB first, so the bit mask is 0b11000000 for the first
 * field and 0b00111111 for the second field.
 *
 * This means that if we have a bit field that crosses a byte boundary and does not start on
 * a byte boundary, the bit masks are like this (for a 16 bit field starting at the 3rd bit):
 *
 * 0b00111111 0b11111111 0b11000000
 *     ------   --------   --
 *     000000   11110000   11
 *     543210   32109876   54
 *
 * So we are forced to mask bits 0 and 1 of the first byte. Since we need to process the previous
 * field first, we cannot repeatedly shift bits out of the byte: if we shift left we get the first
 * field first, but in MSB order. We need bit values in LSB order, as the next byte will be more
 * significant. But we can't shift right as that will give us bits in LSB order but then we get the
 * two fields in the wrong order...
 *
 * So for that reason we explicitly test, per byte, how many bits we need and how many we have already
 * used.
 *
 */

void extractNumber(const Field * field, uint8_t * data, uint32_t startBit, uint32_t bits, int64_t * value, int64_t * maxValue)
{
  bool hasSign = field->hasSign;

  uint32_t firstBit = startBit;
  uint32_t bitsRemaining = bits;
  uint32_t magnitude = 0;
  uint32_t bitsInThisByte;
  uint64_t bitMask;
  uint64_t allOnes;
  uint64_t valueInThisByte;

  *value = 0;
  *maxValue = 0;

  while (bitsRemaining)
  {
    bitsInThisByte = min(8 - firstBit, bitsRemaining);
    allOnes = (uint64_t) ((((uint64_t) 1) << bitsInThisByte) - 1);

    //How are bits ordered in bytes for bit fields? There are two ways, first field at LSB or first
    //field as MSB.
    //Experimentation, using the 129026 PGN, has shown that the most likely candidate is LSB.
    bitMask = allOnes << firstBit;
    valueInThisByte = (*data & bitMask) >> firstBit;

    *value |= valueInThisByte << magnitude;
    *maxValue |= (int64_t) allOnes << magnitude;

    magnitude += bitsInThisByte;
    bitsRemaining -= bitsInThisByte;
    firstBit += bitsInThisByte;
    if (firstBit >= 8)
    {
      firstBit -= 8;
      data++;
    }
  }

  if (hasSign)
  {
    *maxValue >>= 1;

    if (field->offset) /* J1939 Excess-K notation */
    {
      *value += field->offset;
    }
    else
    {
      bool negative = (*value & (((uint64_t) 1) << (bits - 1))) > 0;

      if (negative)
      {
        /* Sign extend value for cases where bits < 64 */
        /* Assume we have bits = 16 and value = -2 then we do: */
        /* 0000.0000.0000.0000.0111.1111.1111.1101 value    */
        /* 0000.0000.0000.0000.0111.1111.1111.1111 maxvalue */
        /* 1111.1111.1111.1111.1000.0000.0000.0000 ~maxvalue */
        *value |= ~*maxValue;
      }
    }
  }
}

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif
char *manufacturer[1 << 12];

void fillManufacturers(void)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(manufacturer); i++) {
		manufacturer[i] = 0;
	}
	for (i = 0; i < ARRAY_SIZE(companyList); i++)
	{
		manufacturer[companyList[i].id] = companyList[i].name;
	}
}

void fillFieldCounts(void)
{
	size_t i, j;

	for (i = 0; i < ARRAY_SIZE(pgnList); i++)
	{
		for (j = 0; pgnList[i].fieldList[j].name && j < ARRAY_SIZE(pgnList[i].fieldList); j++);
		if (j == ARRAY_SIZE(pgnList[i].fieldList))
		{
			//logError("Internal error: PGN %d '%s' does not have correct fieldlist.\n", pgnList[i].pgn, pgnList[i].description);
			exit(2);
		}
		if (j == 0 && pgnList[i].known)
		{
			//logError("Internal error: PGN %d '%s' does not have fields.\n", pgnList[i].pgn, pgnList[i].description);
			exit(2);
		}
		pgnList[i].fieldCount = j;
	}
}





















































