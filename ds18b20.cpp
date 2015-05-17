/**
 * @file ds18b20.cpp
 *
 * @brief DS18B20 driver implementation
 *
 * @author Evgeny Onopchenko
 * @version 1.0
 */


#include "ds18b20.h"


/// CRC8 table
const unsigned char DallasCrc8Table[256] =
{
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};


/// Dallas 1-wire timeouts, us
enum DallasTimeouts
{
	A = 6,
	B = 64,
	C = 60,
	D = 10,
	E = 9,
	F = 55,
	G = 0,
	H = 780,
	I = 70,
	J = 410,
};


/// Constants
enum Constants
{
	DS18B20_FAMILY_CODE = 0x28,				///< Family code
	DS18B20_CONVERT_TIMEOUT_9BIT = 100,		///< Temperature conversion timeout for 9-bit resolution (93.75 ms)
	DS18B20_RES_9BIT = 0x1F,				///< 9-bit resolution
	
	CMD_SEARCH_ROM = 0xF0,					///< Search ROM
	CMD_READ_ROM = 0x33,					///< Read ROM
	CMD_MATCH_ROM = 0x55,					///< Match ROM
	CMD_SKIP_ROM = 0xCC,					///< Skip ROM
	CMD_CONVERT_TMP = 0x44,					///< Convert temperature
	CMD_READ_MEM = 0xBE,					///< Read memory
	CMD_WRITE_SCRATCHPAD = 0x4E,			///< Write scratchpad
	
	SCAN_NET_TIMEOUT = 1000,				///< Dallas network scan timeout
};


/**
* @brief <b> Constructor </b>
*/
DS18B20::DS18B20()
{
	/// Dallas devices addresses chart
	DallasChart = new char* [SensorCount];
	for(uint8_t sensor = 0; sensor < SensorCount; sensor++)
	{
		DallasChart[sensor] = new char [8];
	}
	for(uint8_t sensor = 0; sensor < SensorCount; sensor++)
	{
		for(uint8_t byte = 0; byte < 8; byte++)
		{
			DallasChart[sensor][byte] = 0;
		}
	}
	
	Hw->Set1WBusState(1);
	
	TmpSensorIndex = 0;
	DallasChartSize = 0;
	Step = SCAN_NET;
}


/**
* @brief <b> Execution </b>
*/
void DS18B20::Execute()
{
	switch(Step)
	{
		/// Dallas network scanning
		case SCAN_NET:
		{
			if(ScanNetTimer.Elapsed())
			{
				ScanNetTimer.Start(SCAN_NET_TIMEOUT);
				ScanNet();
			}
			Step = CONVERT_TMP;
			return;
		}
		
		/// Temperature conversion
		case CONVERT_TMP:
		{
			if(TouchReset())
			{
//				WriteByte(CMD_SKIP_ROM);
//				WriteByte(CMD_CONVERT_TMP);
//				ConvertTimer.Start(DS18B20_CONVERT_TIMEOUT_9BIT);
//				Step = READ_TMP;
				
				uint8_t dallasIndex = 0;
				
				for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
				{
					if((DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE) && (dallasIndex++ == TmpSensorIndex))
					{
						if(TouchReset())
						{
							WriteByte(CMD_MATCH_ROM);
							for(uint8_t index = 0; index < 8; index++)
							{
								WriteByte(DallasChart[dallasNum][index]);
							}
							WriteByte(CMD_CONVERT_TMP);
							ConvertTimer.Start(DS18B20_CONVERT_TIMEOUT_9BIT);
							Step = READ_TMP;
							break;
						}
					}
				}
			}
			else
			{
				Step = SCAN_NET;
			}
			
			return;
		}
		
		/// Temperature reading
		case READ_TMP:
		{
			if(ConvertTimer.Elapsed())
			{
				uint8_t dallasIndex = 0;
				
				for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
				{
					if((DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE) && (dallasIndex++ == TmpSensorIndex))
					{
						if(TouchReset())
						{
							WriteByte(CMD_MATCH_ROM);
							for(uint8_t index = 0; index < 8; index++)
							{
								WriteByte(DallasChart[dallasNum][index]);
							}
							WriteByte(CMD_READ_MEM);
							
							uint16_t scratchpad = ReadByte();
							scratchpad |= ReadByte() << 8;
							
							Temperature[TmpSensorIndex] = scratchpad >> 4;
							break;
						}
					}
				}
				
				/// If all devices polled, return to scanning of the network
				/// else continue polling
				if(++TmpSensorIndex == TmpSensorCount)
				{
					TmpSensorIndex = 0;
					Step = SCAN_NET;
				}
				else
				{
					Step = CONVERT_TMP;
				}
			}
			return;
		}
		
		default:
		{
			return;
		}
	}
}


/**
* @brieb <b> Touch and reset bus </b>
* @return true, if, at least, one device present on the bus
*/
bool DS18B20::TouchReset()
{
	Timer::DelayUs(G);
	Hw->Set1WBusState(0);
	Timer::DelayUs(H);
	Core::EnterCritical();
	Hw->Set1WBusState(1);
	Timer::DelayUs(I);
	bool present = !Hw->Get1WBusState();
	Core::ExitCritical();
	Timer::DelayUs(J);
	return present;
}


/**
* @brieb <b> Writing a bit on the bus </b>
* @param bit - bit value for writing
*/
void DS18B20::WriteBit(bool bit)
{
	Core::EnterCritical();
	if(bit)
	{
		Hw->Set1WBusState(0);
		Timer::DelayUs(A);
		Hw->Set1WBusState(1);
		Timer::DelayUs(B);
	}
	else
	{
		Hw->Set1WBusState(0);
		Timer::DelayUs(C);
		Hw->Set1WBusState(1);
		Timer::DelayUs(D);
	}
	Core::ExitCritical();
}


/**
* @brieb <b> Reading a bit from the bus </b>
* @return read bit value
*/
bool DS18B20::ReadBit()
{
	Core::EnterCritical();
	Hw->Set1WBusState(0);
	Timer::DelayUs(A);
	Hw->Set1WBusState(1);
	Timer::DelayUs(E);
	bool value = Hw->Get1WBusState();
	Core::ExitCritical();
	Timer::DelayUs(F);
	return value;
}


/**
* @brieb <b> Writing a byte on the bus </b>
* @param byte - byte value for writing
*/
void DS18B20::WriteByte(uint8_t byte)
{
	for(uint_fast8_t bit = 8;  bit;  bit--)
	{
		WriteBit(byte & 0x01);
		byte >>= 1;
	}
}


/**
* @brieb <b> Reading a byte from the bus </b>
* @return read byte value
*/
uint8_t DS18B20::ReadByte()
{
	uint8_t data = 0;
	for(uint_fast8_t bit = 8;  bit;  bit--)
	{
		data >>= 1;		
		if(ReadBit())
		{
			data |= 0x80;
		}
	}
	return data;
}


/**
* @brief <b> CRC8 calculation  </b>
* @param buffer - data buffer
* @param size - data size
*/
uint8_t DS18B20::CalcCRC(char *buffer, uint16_t size)
{
	uint_fast8_t result = 0x00;
	char *end = buffer + size;
	while(buffer < end)
	{
		result = DallasCrc8Table[result ^ *buffer++];
	}
	return result;
}


/**
* @brief <b> Dallas network scanning </b>
*/
void DS18B20::ScanNet()
{
	/// Address tree route (8 devices)
	uint8_t route[8];
	memset(route, 0, 8);
	
	for(uint8_t sensorNum = 0; sensorNum < SensorCount; sensorNum++)
	{
		if(TouchReset())
		{
			memset(&DallasChart[sensorNum][0], 0, 8);
			
			WriteByte(CMD_SEARCH_ROM);
			
			int8_t forkByte = -1;
			int8_t forkDigit = -1;
			
			for(uint8_t byte = 0; byte < 8; byte++)
			{
				for(uint8_t digit = 0; digit < 8; digit++)
				{
					bool bit0 = ReadBit();
					bool bit1 = ReadBit();
					
					if(!bit0 && bit1)
					{
						/// 0->1 = 0
						WriteBit(0);
					}
					else if(bit0 && !bit1)
					{
						/// 1->0 = 1
						DallasChart[sensorNum][byte] |= 1 << digit;
						WriteBit(1);
					}
					else
					{
						/// Fork
						uint8_t bit = route[byte] & (1 << digit);
						if(bit == 0)
						{
							forkByte = byte;
							forkDigit = digit;
						}
						WriteBit(bit);
						DallasChart[sensorNum][byte] |= bit << digit;
					}
				}
			}
			
			DallasChartSize = sensorNum + 1;
			
			if((forkByte == -1) && (forkDigit == -1))
			{
				/// If no forks, exit
				break;
			}
			else
			{
				/// Else, change the route
				route[forkByte] |= 1 << forkDigit;
			}
		}
	}
	
	/// Check CRC of all devices
	for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
	{
		if(DallasChart[dallasNum][7] != CalcCRC(&DallasChart[dallasNum][0], 7))
		{
			/// CRC error
			memset(&DallasChart[dallasNum][0], 0, 8);
		}
		else
		{
			/// CRC match
		}
	}
	
	/// Find all temperature sensors, set resolution
	TmpSensorCount = 0;
	for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
	{
		if((DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE) && TouchReset())
		{
			TmpSensorCount++;
			
			WriteByte(CMD_MATCH_ROM);
			for(uint8_t index = 0; index < 8; index++)
			{
				WriteByte(DallasChart[dallasNum][index]);
			}
			/// Set resolution
			WriteByte(CMD_WRITE_SCRATCHPAD);
			WriteByte(0x00);					// Tl
			WriteByte(0x00);					// Th
			WriteByte(DS18B20_RES_9BIT);		// Configuration Register
		}
	}
}
