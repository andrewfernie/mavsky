#pragma once

class FrSkySPortPassthru {
private:

public:
	FrSkySPortPassthru();
	// Frsky variables     
	short    crc;                         // of frsky-packet
	uint8_t  time_slot_max = 16;
	uint32_t time_slot = 1;
	float a, az, c, dis, dLat, dLon;
	uint8_t sv_count = 0;

	volatile uint8_t *uartC3;
	enum SPortMode { RX = 0, TX = 1 };
	SPortMode mode, modeNow;

	void setSPortMode(SPortMode mode);
	void FrSkySPort_Init(void);

#if defined Air_Mode || defined Relay_Mode
	void ReadSPort(void);
#endif

#if defined Ground_Mode
	void Emulate_ReadSPort();
#endif

	void FrSkySPort_Process();
	void FrSkySPort_SendByte(uint8_t byte, bool addCrc);
	void CheckByteStuffAndSend(uint8_t byte);
	void FrSkySPort_SendCrc();
	void FrSkySPort_SendDataFrame(uint8_t Instance, uint16_t Id, uint32_t value);
	uint32_t bit32Extract(uint32_t dword, uint8_t displ, uint8_t lth);
	void bit32Pack(uint32_t dword, uint8_t displ, uint8_t lth);
	uint32_t createMask(uint8_t lo, uint8_t hi);
	void SendLat800();
	void SendLon800();
	void SendStatusTextChunk5000();
	void SendAP_Status5001();
	void Send_GPS_Status5002();
	void Send_Bat1_5003();
	void Send_Home_5004();
	void Send_VelYaw_5005();
	void Send_Atti_5006();
	void SendParameters5007();
	void Send_Bat2_5008();
	void Send_WayPoint_5009();
	void Send_Servo_Raw_50F1();
	void Send_VFR_Hud_50F2();
	void Send_Wind_Estimate_50F3();
	void SendRssiF101();
	int8_t PWM_To_63(uint16_t PWM);
	uint32_t Abs(int32_t num);
	float Distance(Loc2D loc1, Loc2D loc2);
	float Azimuth(Loc2D loc1, Loc2D loc2);
	int16_t Add360(int16_t arg1, int16_t arg2);
	float wrap_360(int16_t angle);
	uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);
	//void Send_RC_5009();
}





