/**
   File: dallas.hpp
   
   System:
   Component Name: Dallas temperature sensors driver module
   Status:         Version 1.1 Release 1 

   Language: C++

   Address:
	St.Petersburg, Russia
	  
   Author: Evgeny Onopchenko
   
   E-Mail: jake.spb@gmail.com
   
   Description: Header file for Dallas temperature sensors driver implementation
                This file contains the defined types

*/ 
// ----------------------------------------------------------------------------


#ifndef __DALLAS_HPP
#define __DALLAS_HPP

/**
* @brief <b> Dallas class </b>
*/
class Dallas
{
	public:
		void Execute();									///< Execution
		Dallas();										///< Constructor
	
	private:
		/// Cooperation multitasking steps
		enum Step_t
		{
			SCAN_NET,									///< Dallas network scanning
			CONVERT_TMP,								///< Temperature conversion
			READ_TMP,									///< Temperature reading
		};
		
		Step_t Step;									///< Cooperation multitasking step
		
		Timer ScanNetTimer;								///< Network scanning timer
		
		bool FirstScan;									///< First measure flag
		bool ForceRenew;								///< Neccessity of repeated sensors tunning
		
		uint8_t Route;									///< The current scanning route
		uint8_t RouteMask[16];							///< Visited routes mask
		
		uint8_t DallasChartSize;						///< Count of devices in the network
		char** DallasChart;								///< Devices addresses chart
		uint8_t LastDallasChartSize;					///< Count of devices in the network (for watching changes)
		char** LastDallasChart;							///< Devices addresses chart (for watching changes)
		
		/// Dallas network functions
		bool TouchReset();								///< Reset and check presence
		void WriteBit(bool bit);						///< Writing a bit on the bus
		bool ReadBit();									///< Reading a bit from the bus
		void WriteByte(uint8_t byte);					///< Writing a byte on the bus
		uint8_t ReadByte();								///< Reading a byte from the bus
		uint8_t CalcCRC(char* buffer, uint16_t size);	///< CRC8 calculation
		bool ScanNet();									///< Dallas network scanning
		
		Timer ConvertTimer;								///< Temperature conversion timer
		uint8_t TmpSensorCount;							///< Temperature sensors count
		uint8_t TmpSensorIndex;							///< Temperature sensor number
};

#endif // __DALLAS_HPP
