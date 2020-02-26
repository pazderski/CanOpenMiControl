#pragma once

#include "sdo.hpp"
#include "pdo.hpp"
#include "nmt.hpp"


namespace MiControlCmds
{

enum NumberOfBits {DataSize4 = 4, DataSize8 = 8, DataSize16 = 16, DataSize32 = 32};

inline SdoCmd ClearError()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	//cmd.index = __builtin_bswap16(0x3000);
	cmd.index = 0x3000;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0x01;
	cmd.timeout = 30;
	cmd.trials = -1;
	return cmd;
}

// numberOfBits: Only 8, 16 and 32 available.
inline SdoCmd MapRPDO(int object, int index, int subIndex, NumberOfBits numberOfBits)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1600;
	cmd.subindex = object;
	cmd.data[0] = numberOfBits;
	cmd.data[1] = subIndex;
	cmd.data[2] = index;
	cmd.data[3] = index >> 8;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

// Alternatywna sk�adnia mapowania RPDO.
inline SdoCmd MapRPDO(int object, const SdoCmd &c, NumberOfBits numberOfBits)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1600;
	cmd.subindex = object;
	cmd.data[0] = numberOfBits;
	cmd.data[1] = c.subindex;
	cmd.data[2] = c.index;
	cmd.data[3] = c.index >> 8;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd EnableRPDO(int numberOfObjects)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1600;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = numberOfObjects;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd DisableRPDO()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1600;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd SetTransmissionType(int channel)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1800 + channel;
	cmd.subindex = 0x02;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0x01;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd Map02hTxPDO(int object, int index, int subIndex, int numberOfBits)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1A00 + object; // TPDO communication parameter - highest sub-index supported
	cmd.subindex = 02;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = numberOfBits;
	cmd.data[1] = subIndex;
	cmd.data[2] = index;
	cmd.data[3] = index >> 8;
	cmd.timeout = 20;
	cmd.trials = -1;
	return cmd;
}

// numberOfBits: Only 8, 16 and 32 available.
inline SdoCmd MapTPDO(int object, int index, int subIndex, int numberOfBits)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1A00 + object;
	cmd.subindex = 01;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = numberOfBits;
	cmd.data[1] = subIndex;
	cmd.data[2] = index;
	cmd.data[3] = index >> 8;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

// Alternatywna sk�adnia mapowania RPDO.
inline SdoCmd MapTPDO(int object, const SdoCmd &c, NumberOfBits numberOfBits)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1A00 + object;
	cmd.subindex = 01;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = numberOfBits;
	cmd.data[1] = c.subindex;
	cmd.data[2] = c.index;
	cmd.data[3] = c.index >> 8;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd EnableTPDO(uint8_t pdoNum, int numberOfObjects)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1A00 + pdoNum - 1;
	cmd.subindex = 0;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = numberOfObjects;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd DisableTPDO(int numberOfObjects)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x1A00 + numberOfObjects;
	cmd.subindex = 0;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd NMTOperational()
{
	SdoCmd cmd;
	cmd.type = 0x01;
	cmd.index = 0x0001;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}


inline SdoCmd LoadDefault()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3000;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0x82;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd RestoreParam()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3000;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0x81;
	cmd.timeout = 30;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd SetMotorDC()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3900;
	cmd.subindex = 0x00;
	cmd.data[0] = 0x00;
	cmd.timeout = 20;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd SetMotorBLDC()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3900;
	cmd.subindex = 0x00;
	cmd.data[0] = 0x01;
	cmd.timeout = 20;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd MotorEnable()
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3004;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0x01;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd MotorDisable()
{

	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3004;
	cmd.subindex = 0x00;
	cmd.data[0] = cmd.data[1] = cmd.data[2] = cmd.data[3] = 0;
	cmd.data[0] = 0x00;
	cmd.timeout = 10;
	cmd.trials = -1;
	return cmd;
}

inline SdoCmd SetSubvel(int32_t value)
{
	SdoCmd cmd;
	cmd.type = 0x23;
	cmd.index = 0x3500;
	cmd.subindex = 0x00;
	// Is it possible to use something like this: "*(uint32_t*)cmd.data = value" ?
	cmd.data[0] = value;
	cmd.data[1] = value >> 8;
	cmd.data[2] = value >> 16;
	cmd.data[3] = value >> 24;
	cmd.timeout = 5;
	cmd.trials = -1;
	return cmd;
}
};
