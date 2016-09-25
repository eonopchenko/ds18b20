/**
 * @file dallas.cpp
 *
 * @brief Dallas temperature sensors driver implementation
 *
 * @author Evgeny Onopchenko
 * @version 1.1
 */


#include "dallas.hpp"
#include <string.h>
#include <stdio.h>


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
	MAX_SENSOR_COUNT = 8,					///< Maximum sensor count
	
	DS18B20_FAMILY_CODE = 0x28,				///< Family code
	DS1820_FAMILY_CODE = 0x10,				///< Family code
	DS1820_CONVERT_TIMEOUT = 750,			///< 
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
Dallas::Dallas()
{
	/// Dallas devices addresses chart
	DallasChart = new char* [MAX_SENSOR_COUNT];
	LastDallasChart = new char* [MAX_SENSOR_COUNT];
	for(uint8_t sensor = 0; sensor < MAX_SENSOR_COUNT; sensor++)
	{
		DallasChart[sensor] = new char [8];
		LastDallasChart[sensor] = new char [8];
	}
	for(uint8_t sensor = 0; sensor < MAX_SENSOR_COUNT; sensor++)
	{
		for(uint8_t byte = 0; byte < 8; byte++)
		{
			DallasChart[sensor][byte] = 0;
			LastDallasChart[sensor][byte] = 0;
		}
	}
	
	Hw->Set1WBusState(1);
	
	TmpSensorIndex = 0;
	DallasChartSize = 0;
	LastDallasChartSize = 0;
	Step = SCAN_NET;
	FirstScan = true;
	ForceRenew = false;
	Route = 0;
}


/**
* @brief <b> Execution </b>
*/
void Dallas::Execute()
{
	switch(Step)
	{
		/// Dallas network scanning
		case SCAN_NET:
		{
			if(ScanNetTimer.Elapsed() && ScanNet())
			{
				ScanNetTimer.Start(SCAN_NET_TIMEOUT);
				Step = CONVERT_TMP;
			}
			return;
		}
		
		/// Temperature conversion
		case CONVERT_TMP:
		{
			if(DallasChartSize && TouchReset())
			{
//				WriteByte(CMD_SKIP_ROM);
//				WriteByte(CMD_CONVERT_TMP);
//				ConvertTimer.Start(DS18B20_CONVERT_TIMEOUT_9BIT);
//				Step = READ_TMP;
				
				bool found = false;
				uint8_t dallasIndex = 0;
				
				for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
				{
					if(((DallasChart[dallasNum][0] == DS1820_FAMILY_CODE) || 
						(DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE)) 
						&& (dallasIndex++ == TmpSensorIndex))
					{
						found = true;
						if(TouchReset())
						{
							WriteByte(CMD_MATCH_ROM);
							for(uint8_t index = 0; index < 8; index++)
							{
								WriteByte(DallasChart[dallasNum][index]);
							}
							WriteByte(CMD_CONVERT_TMP);
							Target->SetOneWireData(true);
							
							ConvertTimer.Start((
								DallasChart[dallasNum][0] == DS1820_FAMILY_CODE) ? 
								DS1820_CONVERT_TIMEOUT : 
								DS18B20_CONVERT_TIMEOUT_9BIT);
							
							Step = READ_TMP;
							break;
						}
					}
				}
				
				/// Is sensor found?
				if(!found)
				{
					TmpSensorIndex = 0;
					Step = SCAN_NET;
					return;
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
			if(DallasChartSize == 0)
			{
				Step = SCAN_NET;
				return;
			}
			
			if(ConvertTimer.Elapsed())
			{
				bool found = false;
				uint8_t dallasIndex = 0;
				
				for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
				{
					if(((DallasChart[dallasNum][0] == DS1820_FAMILY_CODE) || 
						(DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE)) && 
						(dallasIndex++ == TmpSensorIndex))
					{
						found = true;
						if(TouchReset())
						{
							WriteByte(CMD_MATCH_ROM);
							for(uint8_t index = 0; index < 8; index++)
							{
								WriteByte(DallasChart[dallasNum][index]);
							}
							WriteByte(CMD_READ_MEM);
							
							/// Recognize type of sensor
							int16_t temperature = 0x7FFF;
							if(DallasChart[dallasNum][0] == DS1820_FAMILY_CODE)
							{
								char scratchpad[9];
								for(uint8_t index = 0; index < 9; index++)
								{
									scratchpad[index] = ReadByte();
								}
								
								/// If crc is correct, memorize temperature
								if(GetCRC(scratchpad, 8) == scratchpad[8])
								{
									temperature = (scratchpad[1] << 8) | scratchpad[0];
									temperature >>= 1;
								}
							}
							else if(DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE)
							{
								temperature = ReadByte();
								temperature |= ReadByte() << 8;
								temperature >>= 4;
							}
							
							if(temperature < 85)
							{
								/// DallasChart[TmpSensorIndex] = Sensor address
								/// temperature = Actual temperature
							}
							else
							{
								ForceRenew = true;
								TmpSensorIndex = 0;
								Step = SCAN_NET;
								return;
							}
							break;
						}
					}
				}
				
				/// Check, whether a sensor is found
				if(!found)
				{
					TmpSensorIndex = 0;
					Step = SCAN_NET;
					return;
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
bool Dallas::TouchReset()
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
void Dallas::WriteBit(bool bit)
{
	Core::EnterCritical();
	{
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
	}
	Core::ExitCritical();
}


/**
* @brieb <b> Reading a bit from the bus </b>
* @return read bit value
*/
bool Dallas::ReadBit()
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
void Dallas::WriteByte(uint8_t byte)
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
uint8_t Dallas::ReadByte()
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
uint8_t Dallas::CalcCRC(char *buffer, uint16_t size)
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
* @return true if the entire network is scanned
*/
bool Dallas::ScanNet()
{
	/// If it is the first route, set to zero chart size and route mask
	if(Route == 0)
	{
		memset(RouteMask, 0, 16);
		DallasChartSize = 0;
	}
	
	/// Memorize the current chart size
	/// (to limit the maximum routes for 1 time)
	uint8_t dallasChartSize = DallasChartSize;
	
	/// Declare fork counter
	/// (for filling the mask and definition if sensor is the only one)
	uint8_t fork = 0xFF;
	
	/// Search cycle
	while((DallasChartSize - dallasChartSize < 4) && (Route < 16) && (DallasChartSize < MAX_SENSOR_COUNT) && (fork > 0))
	{
		if(TouchReset())
		{
			memset(&DallasChart[DallasChartSize][0], 0, 8);
			
			/// Set for counter to zero
			fork = 0;
			
			WriteByte(CMD_SEARCH_ROM);
			
			for(uint8_t byteNum = 0; byteNum < 8; byteNum++)
			{
				for(uint8_t bitNum = 0; bitNum < 8; bitNum++)
				{
					bool value0 = ReadBit();
					bool value1 = ReadBit();
					
					/// 0 -> 1 = 0
					if((value0 == 0) && (value1 == 1))
					{
						DallasChart[DallasChartSize][byteNum] &= ~(1 << bitNum);
						WriteBit(0);
					}
					
					/// 1 -> 0 = 1
					else if((value0 == 1) && (value1 == 0))
					{
						DallasChart[DallasChartSize][byteNum] |= (1 << bitNum);
						WriteBit(1);
					}
					
					/// Fork
					else
					{
						/// Renew route mask
						RouteMask[Route] |= (1 << fork);
						
						/// Define transition type
						if(Route & (1 << fork))
						{
							DallasChart[DallasChartSize][byteNum] |= (1 << bitNum);
							WriteBit(1);
						}
						else
						{
							DallasChart[DallasChartSize][byteNum] &= ~(1 << bitNum);
							WriteBit(0);
						}
						
						fork++;
					}
				}
			}
			
			DallasChartSize++;
			
			/// Check the route for doubline (to avoid visiting of the same route many times)
			/// limit - the number of current route (to avoid out limits)
			/// route - visited routes counter
			uint8_t limit = Route;
			uint8_t route = 0;
			
			Route++;
			
			while(Route < 16)
			{
				/// route > limit means that
				/// the last doubling was checked
				if(route > limit)
				{
					break;
				}
				
				/// If the route is doubling, increment the number of current route, set counter to zero and continue
				if((Route & RouteMask[route]) == (route & RouteMask[route]))
				{
					Route++;
					route = 0;
					continue;
				}
				
				route++;
			}
		}
		else
		{
			break;
		}
	}
	
	/// Check the exit condition
	if((Route < 16) && (DallasChartSize < MAX_SENSOR_COUNT) && (fork > 0) && (fork != 0xFF))
	{
		return false;
	}
	
	/// Set the number of current route to zero
	Route = 0;
	
	/// Check CRC of all devices in the network
	for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
	{
		if(DallasChart[dallasNum][7] != GetCRC(&DallasChart[dallasNum][0], 7))
		{
			/// CRC error
			memset(&DallasChart[dallasNum][0], 0, 8);
		}
		else
		{
			/// CRC ok
		}
		
		/// Check the error of reading the identifier
		uint8_t zeros[] = {0, 0, 0, 0, 0, 0, 0, 0};
		if(!memcmp(&DallasChart[dallasNum][0], zeros, 8))
		{
			return false;
		}
	}
	
	/// Check, whether the chart changed
	bool renew = DallasChartSize != LastDallasChartSize;
	
	if(!renew)
	{
		for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
		{
			if(memcmp(&DallasChart[dallasNum][0], &LastDallasChart[dallasNum][0], 8))
			{
				renew = true;
				break;
			}
		}
	}
	
	/// Memorize new chart
	LastDallasChartSize = DallasChartSize;
	for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
	{
		memcpy(&LastDallasChart[dallasNum][0], &DallasChart[dallasNum][0], 8);
	}
	
	/// If something changed in the chart
	if(renew || ForceRenew)
	{
		ForceRenew = false;
		
		/// Find all temperature sensors and tune resolutions
		TmpSensorCount = 0;
		for(uint8_t dallasNum = 0; dallasNum < DallasChartSize; dallasNum++)
		{
			if((DallasChart[dallasNum][0] == DS1820_FAMILY_CODE) || 
				(DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE))
			{
				TmpSensorCount++;
				
				if((DallasChart[dallasNum][0] == DS18B20_FAMILY_CODE) && TouchReset())
				{
					WriteByte(CMD_MATCH_ROM);
					for(uint8_t index = 0; index < 8; index++)
					{
						WriteByte(DallasChart[dallasNum][index]);
					}
					
					/// Tune resolution
					WriteByte(CMD_WRITE_SCRATCHPAD);
					WriteByte(0x00);					// Tl
					WriteByte(0x00);					// Th
					WriteByte(DS18B20_RES_9BIT);		// Configuration Register
				}
			}
		}
		
		/// If the first scan
		if(FirstScan)
		{
			FirstScan = false;
			
			if(TmpSensorCount)
			{
				/// Memorize sensor address
				/// DallasChart[tmpSensorNum] = Sensor address
				tmpSensorNum++
			}
		}
	}
	
	return true;
}
