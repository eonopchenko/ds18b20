/**
   File: ds18b20.h
   
   System:
   Component Name: DS18B20 driver module
   Status:         Version 1.0 Release 1 

   Language: C++

   Address:
	St.Petersburg, Russia
	  
   Author: Evgeny Onopchenko
   
   E-Mail: jake.spb@gmail.com
   
   Description: Header file for DS18B20 temperature sensor driver implementation
                This file contains the defined types

*/ 
// ----------------------------------------------------------------------------


#ifndef __DS18B20_H
#define __DS18B20_H

/**
* @brief <b> DS18B20 </b>
*/
class DS18B20
{
	public:
		void Execute();									///< Execution
		DS18B20();										///< Constructor
	
	private:
		enum Step_t
		{
			SCAN_NET,									///< Dallas network scanning
			CONVERT_TMP,								///< DS18B20 temperature conversion
			READ_TMP,									///< DS18B20 temperature reading
		};
		
		Step_t Step;									///< Cooperation multitasking step
		
		Timer ScanNetTimer;								///< Network scanning timer
		
		uint8_t DallasChartSize;						///< Dallas network addresses count
		char** DallasChart;								///< Dallas network addresses chart
		
		/// Dallas network functions
		bool ResetAndTouch();							///< Reset and check presence
		void WriteBit(bool bit);						///< Writing a bit on the bus
		bool ReadBit();									///< Reading a bit from the bus
		void WriteByte(uint8_t byte);					///< Writing a byte on the bus
		uint8_t ReadByte();								///< Reading a byte from the bus
		uint8_t CalcCRC(char *buffer, uint16_t size);	///< CRC8 calculation
		void ScanNet();									///< Dallas network scanning
		
		SoftTimer ConvertTimer;							///< Temperature conversion timer
		uint8_t TmpSensorCount;							///< Temperature sensors count
		uint8_t TmpSensorIndex;							///< Temperature sensor number
};

#endif
