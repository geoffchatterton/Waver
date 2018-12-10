#include <SPI.h>
#include <RH_RF95.h>

#include <Wire.h>
#include "BMA250.h"

#define SERIAL_ON    0

#if SERIAL_ON
#define PRINT(x)    Serial.print(x)
#define PRINTLN(x)  Serial.println(x)
#else
#define PRINT(x)
#define PRINTLN(x)
#endif

/*******************************************************/

/* for feather M0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBATPIN A7

// Change to 434.0 or other frequency, but must match RX freq!
#define RF95_FREQ 433.0

#define MAX_PACKET_CT 4
typedef struct {
    uint8_t     serial;
    uint16_t    vbat;
    int16_t    ax[MAX_PACKET_CT];
    int16_t    ay[MAX_PACKET_CT];
    int16_t    az[MAX_PACKET_CT];
} wave_data_packet;  // max length of packet message is 251 bytes

wave_data_packet wdp;
int16_t cx, cy, cz;
int sample_ct;
int packet_ct;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the accelerometer driver
BMA250 accel;

/*******************************************************/

void print_wdp()
{
	int i;

	uint8_t* buf = (uint8_t*)&wdp;
	
	for(i = 0; i < sizeof(wave_data_packet); i++)
	{
		PRINT(buf[i]);
		PRINT(" ");
	}
	PRINTLN();

	PRINT(wdp.serial);
	PRINT("\t");
	PRINT(wdp.vbat);
	PRINT("\t");
	
	for(i = 0; i < MAX_PACKET_CT; i++)
	{
		PRINT(wdp.ax[i]);
		PRINT("\t");
		PRINT(wdp.ay[i]);
		PRINT("\t");
		PRINT(wdp.az[i]);
		PRINT(",\t");
	}
	PRINTLN();
}

/*******************************************************/

void setup()
{
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    
#if SERIAL_ON
    Serial.begin(115200);
    while (!Serial) {
        delay(1);
    }
#endif
    sample_ct = 0;
    packet_ct = 0;
    
    wdp.serial = 0;
    wdp.vbat = 0;
    int i;
    for(i = 0; i < MAX_PACKET_CT; i++)
    {
        wdp.ax[i] = 0;
        wdp.ay[i] = 0;
        wdp.az[i] = 0;
    }
    
    Wire.begin();
    accel.begin(BMA250_range_2g, BMA250_update_time_64ms);
    delay(100);
    
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    
    while (!rf95.init()) {
        PRINTLN("LoRa radio init failed");
        while (1);
    }
    PRINTLN("LoRa radio init OK!");
    
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        PRINTLN("setFrequency failed");
        while (1);
    }
    PRINT("Set Freq to: "); PRINTLN(RF95_FREQ);
    rf95.setTxPower(23, false);
}

/*******************************************************/

void loop()
{
    delay(100);
    
    accel.read();
    cx += accel.X;
    cy += accel.Y;
    cz += accel.Z;
    sample_ct += 1;
    
    if(sample_ct == 10)
    {
//    	PRINT(cx); PRINT("\t"); PRINT(cy); PRINT("\t"); PRINTLN(cz);
        wdp.ax[packet_ct] = (cx + 5) / 10;
        wdp.ay[packet_ct] = (cy + 5) / 10;
        wdp.az[packet_ct] = (cz + 5) / 10;
        
        cx = cy = cz = 0;
        sample_ct = 0;
        packet_ct += 1;
        
        if(packet_ct == MAX_PACKET_CT)
        {
            float vbat = analogRead(VBATPIN);
            vbat *= 6.6;
            
            wdp.serial += 1;
            wdp.vbat = (int)vbat;
            packet_ct = 0;
            
#if SERIAL_ON
			print_wdp();
#endif
            rf95.send((uint8_t *)&wdp, sizeof(wave_data_packet));
            delay(10);
            rf95.waitPacketSent();
        }
    }
}
