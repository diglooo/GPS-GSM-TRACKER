#include<NeoSWSerial.h>
#include <AltSoftSerial.h>
#include <LowPower.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

#define MPU6050_INTERRUPT_PIN	2
#define SIM800L_RESET_PIN		5
#define SIM800L_RX_PIN			6
#define SIM800L_TX_PIN			7
#define GPS_RX_PIN				8
#define GPS_TX_PIN				9
#define GPS_PWR_PIN				12
#define VBAT_MEAS_PIN			A3
#define MPU6050_SDA_PIN			A4
#define MPU6050_SCL_PIN			A5

#define SIGNAL_PATH_RESET 104
#define INT_PIN_CFG 55
#define ACCEL_CONFIG 28
#define ACCEL_CONFIG2 29
#define WOM_THR 31 // Motion detection threshold bits [7:0]
#define MOT_DUR 32 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ACCEL_INTEL_CTRL 105
#define INT_ENABLE 56
#define WHO_AM_I_MPU6050 117 // Should return 0x68
#define INT_STATUS 58
#define PWR_MGMT_1 107
#define MPU6050_ADDRESS 104
#define INT_CLEAR 0x3a
#define WOM_ENABLE 6

#define SLEEP_MODE_WDOG_INTERVAL_SEC 600
#define WAKE_MODE_WDOG_INTERVAL_MS 60000
#define GPS_POLL_INTERVAL_MS 10000

#define GSM_CELL_DATA_UPDATE_SLEEP_INTERVAL 6
#define GPS_FIX_MAX_ATTEMPTS_BEFORE_SLEEP 50
#define GPS_MAX_POS_UPDATES_BEFORE_SLEEP 50

AltSoftSerial gpsPort;
//NMEAGPS  NMEAParser; // This parses the GPS characters
//gps_fix  gps_fix_data; // This holds on to the latest values

TinyGPSPlus gps;

#define DEV_TEST
#ifdef DEV_TEST
  NeoSWSerial SIM800LSerial(SIM800L_RX_PIN,SIM800L_TX_PIN);
  #define DEBUG_PORT Serial
#else
  #define SIM800LSerial Serial
#endif

/*
Operator:"TELECOM ITALIA MOBILE" - Operator="TELECOM ITALIA MOBILE"
MCC=222     dec
MNC=01      dec
Rxlev=55    dec
Cellid=07FD hex
Arfcn=1003  dec
Lac=2777    hex
Bsic=14     hex
 */

#define MAX_CELLS 8
int GSMCellTowerCount=0;
struct GSMCellTower{
  uint16_t cellId;
  uint16_t lac;
  uint16_t mcc;
  uint16_t mnc;
};

static GSMCellTower cellTowers[MAX_CELLS];
static char serializedCells[16*MAX_CELLS];

enum APP_STA { STARTUP, SLEEP, GPS_ON, WAIT_FOR_FIX, HAVE_FIX, GPS_COMM_ERR, AUTO_POS_REPORT};
int schedulePositionUpdate = 0;
int battLevel=0;
float gps_latitude, gps_longitude, zAccel;
uint16_t positionUpdatesCounter=1;
long previousMillis = 0; 
bool schedSleep=false;
int BlynkWatchdog = 0;
unsigned long timer_gps_cur = 0, timer_gps_sta=0;
unsigned long timer_wdog_cur = 0, timer_wdog_sta = 0;
int APP_STATE = STARTUP, LAST_APP_STATE = -1;
int motionTriggered = 0;
int motionTriggersCount = 0;
int gpsFixAttempts = 0;
int gprsInitialized = 0;
float vBatt;
uint16_t serialCommAttempts = 0;
int wakeCounts=10;
char tempStr[256];

void requestandParseCellTowers()
{
  DEBUG_PORT.println("GETTING CELL TOWER DATA...");
  flushSIM800Buffer();
  
  SIM800LSerial.println(F("AT"));
  delay(200);

  SIM800LSerial.println(F("AT+CSCLK=0"));
  delay(200);

  SIM800LSerial.println(F("AT+CNETSCAN=1"));
  delay(200);

  flushSIM800Buffer();
  
  //THIS ANSWER TAKES ~15 SECONDS
  SIM800LSerial.println(F("AT+CNETSCAN"));

  int lineCounter=0;
  do
  {
    SIM800LSerial.setTimeout(30000); 
    CreadStringUntil(tempStr,0x0D,256);   
    //DEBUG_PORT.print( strlen(tempStr)); DEBUG_PORT.print(" *** ");
    //DEBUG_PORT.println(tempStr);

    if(lineCounter>1 && strlen(tempStr)<10) break;

    if(strlen(tempStr)>=10)
    {
      static char* fields[16];
      int index=0;
      char* token=0;
      token = strtok(tempStr, ","); 
      while(token != NULL)
      {
          fields[index] = token;       
          index++;
          token = strtok(NULL, ",");
      }
      
      for(int n = 0; n < index; n++)
      { 
        char* subfields[3];
        char* subtoken=0;
        int subindex=0;
        subtoken = strtok(fields[n], ":"); 
        while(subtoken != NULL)
        {
          subfields[subindex] = subtoken;
   
          subindex++;
          subtoken = strtok(NULL, ":");
        }
         
        switch(n)
        {
         case 0: break;//nom
         case 1: cellTowers[GSMCellTowerCount].mcc=atoi(subfields[1]); break; //mcc
         case 2: cellTowers[GSMCellTowerCount].mnc=atoi(subfields[1]); break;//mnc
         case 3: break;//rxlev
         case 4: cellTowers[GSMCellTowerCount].cellId=strtol(subfields[1],0,16); break;//cellid
         case 5: break;//arfcn
         case 6: cellTowers[GSMCellTowerCount].lac=strtol(subfields[1],0,16); break;//lac
         case 7: break; //bsic         
        }
          
      }
      GSMCellTowerCount++;
    }
    lineCounter++;
  }
  while(GSMCellTowerCount<MAX_CELLS);
  flushSIM800Buffer();
  SIM800LSerial.setTimeout(1000); 
  
  DEBUG_PORT.print("FINISH, ");  DEBUG_PORT.print(GSMCellTowerCount);
  DEBUG_PORT.println(" CELLS FOUND.");
}

void CreadStringUntil(char* buff, char term, int max_len)
{
  int cnt=0;
  for (unsigned long start = millis(); (millis() - start < 30000) && (cnt < max_len-2);)
  {
    if(SIM800LSerial.available()) 
    {
      buff[cnt]=SIM800LSerial.read();
      if(buff[cnt]==term) break;
      cnt++; 
    } 
  }
  buff[cnt]=0;
}

void printCells()
{
  for(int i=0;i<GSMCellTowerCount;i++)
  {
    Serial.print("***CELL "); Serial.println(i); 
    Serial.print("MCC="); Serial.println(cellTowers[i].mcc); 
    Serial.print("MNC="); Serial.println(cellTowers[i].mnc); 
    Serial.print("cellId="); Serial.println(cellTowers[i].cellId); 
    Serial.print("lac="); Serial.println(cellTowers[i].lac); 
    Serial.println();
  }
}

void serializeCells()
{
  for(int i=0;i<GSMCellTowerCount;i++)
  {
    sprintf(serializedCells,"%s%04X%04X%04X%04X",serializedCells,cellTowers[i].mcc,cellTowers[i].mnc,cellTowers[i].cellId,cellTowers[i].lac);
  }
}

void writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) 
{
	Wire.begin();
	Wire.beginTransmission(deviceAddress); // Initialize the Tx buffer
	Wire.write(registerAddress); // Put slave register address in Tx buffer
	Wire.write(data); // Put data in Tx buffer
	Wire.endTransmission(); // Send the Tx buffer
}

uint8_t readByte(uint8_t deviceAddress, uint8_t registerAddress) 
{
	uint8_t data;
	Wire.beginTransmission(deviceAddress); 
	Wire.write(registerAddress);
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(deviceAddress, (uint8_t)1); // Read one byte from slave register address 
	data = Wire.read(); // Fill Rx buffer with result
	return data; // Return data read from slave register
}

void setup_MPU6050()
{
	writeByte(MPU6050_ADDRESS, 0x6B, 0x00);
	writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07); 
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  0b00001001);
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG2, 0b00001000);
	writeByte(MPU6050_ADDRESS, WOM_THR, 5); 
	writeByte(MPU6050_ADDRESS, MOT_DUR, 40); 
	writeByte(MPU6050_ADDRESS, ACCEL_INTEL_CTRL, 0b10000000);
	writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x00010000); // INT pin is active low
}

float getZaccel()
{
	uint16_t accH = readByte(MPU6050_ADDRESS, 0X3F);
	uint16_t accL = readByte(MPU6050_ADDRESS, 0X40);
	int16_t result = (accL + accH * 256);
	return -(result / 32767.0);
}

void gpsPowerUp()
{
	pinMode(GPS_PWR_PIN, OUTPUT);
	digitalWrite(GPS_PWR_PIN, HIGH);
}

void gpsPowerDown()
{
	pinMode(GPS_PWR_PIN, OUTPUT);
	digitalWrite(GPS_PWR_PIN, LOW);
}

char StrContains(char *str, char *sfind)
{
	char found = 0;
	char index = 0;
	char len;

	len = strlen(str);

	if (strlen(sfind) > len) {
		return 0;
	}
	while (index < len) {
		if (str[index] == sfind[found]) {
			found++;
			if (strlen(sfind) == found) {
				return 1;
			}
		}
		else {
			found = 0;
		}
		index++;
	}

	return 0;
}

void sim800initGPRS()
{
  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT"));
	delay(200);
 
  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT"));
	delay(200);

  SIM800LSerial.flush();
	SIM800LSerial.println(F("ATE0"));
	delay(200);
	SIM800LSerial.flush();

  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT"));
	waitForOK();
	delay(200);

  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT+CSCLK=0"));
	waitForOK();
	delay(200);

  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT+SAPBR=3,1,\"APN\",\"TM\""));
	waitForOK();
	delay(200);

  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""));
	waitForOK();
	delay(200);

  SIM800LSerial.flush();
	SIM800LSerial.println(F("AT+SAPBR=1,1"));
	waitForOK();
	delay(2000);

	gprsInitialized = 1;
}

void sim800makeHTTPreq(char* url)
{
  flushSIM800Buffer();
	SIM800LSerial.println(F("AT+HTTPINIT"));
	waitForOK();
	delay(200);

  flushSIM800Buffer();
	SIM800LSerial.println(F("AT+HTTPPARA=\"CID\",1"));
	waitForOK();
	delay(200);

  flushSIM800Buffer();
  SIM800LSerial.print("AT+HTTPPARA=\"URL\",\"");
  SIM800LSerial.print(url);
  SIM800LSerial.println("\"");
  waitForOK();
  delay(200);

  flushSIM800Buffer();
	SIM800LSerial.println(F("AT+HTTPACTION=0"));
	echoSIM800Buffer();
	delay(100);

  flushSIM800Buffer();
	SIM800LSerial.println(F("AT+HTTPTERM"));
	waitForOK();
	delay(200);
}

void sim800deinitGPRS()
{
	SIM800LSerial.println(F("AT+SAPBR=0,1\n"));
	waitForOK();
	delay(200);

	SIM800LSerial.println(F("AT+CSCLK=2\n"));
	waitForOK();
	delay(200);

	gprsInitialized = 0;
}

void flushSIM800Buffer()
{
	for (unsigned long start = millis(); millis() - start < 2000;)
	{
		while (SIM800LSerial.available()) SIM800LSerial.read();
	}
}

void echoSIM800Buffer()
{
	for (unsigned long start = millis(); millis() - start < 5000;)
	{
		while (SIM800LSerial.available()) DEBUG_PORT.write(SIM800LSerial.read());
	}
}

void waitForOK()
{
	DEBUG_PORT.print("wait for OK.. ");
	static char tmp[16];
	int i = 0;
	for (unsigned long start = millis(); (millis() - start < 3000) && i<14;)
	{
		while (SIM800LSerial.available())
		{
			char c = SIM800LSerial.read();
			tmp[i] = c;
			i++;
			tmp[i] = 0;
			if (StrContains(tmp, "OK"))
			{
				DEBUG_PORT.println(" OK");
				return;
			}
		}
	}
	if (i<14) DEBUG_PORT.println(" TOUT or ERROR");
}

void waitForHTTPACTION()
{
	DEBUG_PORT.print("wait for HTTPACTION.. ");
	static char tmp[32];
	int i = 0;
	for (unsigned long start = millis(); (millis() - start < 10000) && i < 30;)
	{
		while (SIM800LSerial.available())
		{
			char c = SIM800LSerial.read();
			tmp[i] = c;
			i++;
			tmp[i] = 0;
			if (StrContains(tmp, "+HTTPACTION:"))
			{
				DEBUG_PORT.println(" OK");
				return;
			}
		}
	}
	if (i < 30) DEBUG_PORT.println(" TOUT or ERROR");
}

void updateStatusLocation(float lat, float lon, float zAcc, float vbatt, int status)
{
	static char latString[10];
	static char lonString[10];
	static char zAccString[8];
	static char vBattString[8];
	static char stateString[16];	
	switch (status)
	{
		case STARTUP:		sprintf(stateString, ("IDLE")); break;
		case SLEEP:			sprintf(stateString, ("SLEEP")); break;
		case WAIT_FOR_FIX:	sprintf(stateString, ("WAIT_FOR_FIX")); break;
		case HAVE_FIX:		sprintf(stateString, ("HAVE_FIX")); break;
		case GPS_COMM_ERR:	sprintf(stateString, ("GPS_COMM_ERR")); break;
	}

  //send GPS position data
	dtostrf(lat, 7, 5, latString);
	dtostrf(lon, 7, 5, lonString);
	dtostrf(zAcc, 3, 2, zAccString);
	dtostrf(vbatt, 3, 2, vBattString);
	sprintf(tempStr, "cipolla.ddns.net/?la=%s&lo=%s&za=%s&st=%s&vb=%s", latString, lonString, zAccString, stateString, vBattString);
	sim800makeHTTPreq(tempStr);      
}

void updateGSMCellData(float zAcc,float vbatt, int status, char* serCell)
{
  static char zAccString[8];
  static char vBattString[8];
  static char stateString[16];

  switch (status)
  {
    case STARTUP:   sprintf(stateString, ("IDLE")); break;
    case SLEEP:     sprintf(stateString, ("SLEEP")); break;
    case WAIT_FOR_FIX:  sprintf(stateString, ("WAIT_FOR_FIX")); break;
    case HAVE_FIX:    sprintf(stateString, ("HAVE_FIX")); break;
    case GPS_COMM_ERR:  sprintf(stateString, ("GPS_COMM_ERR")); break;
  }
  
  dtostrf(vbatt, 3, 2, vBattString);
  dtostrf(zAcc, 3, 2, zAccString);
  sprintf(tempStr, "cipolla.ddns.net/?za=%s&vb=%s&st=%s&cd=%s",zAccString,vBattString,stateString,serCell);
  sim800makeHTTPreq(tempStr);
}

void MPU6050_ISR()
{
	motionTriggered = 1;
}

void setup() 
{
	gpsPowerDown();
	pinMode(VBAT_MEAS_PIN, INPUT);
	pinMode(SIM800L_RESET_PIN,OUTPUT);
	digitalWrite(SIM800L_RESET_PIN, LOW);
	delay(200);
	digitalWrite(SIM800L_RESET_PIN, HIGH);
	delay(200);

	DEBUG_PORT.begin(115200);
	gpsPort.begin(9600);
	SIM800LSerial.begin(9600);

  DEBUG_PORT.println();
  DEBUG_PORT.print(F("WAIT FOR SIM800 INIT..."));
  delay(15000);
  DEBUG_PORT.println(F(" DONE"));

/*
	while (1)
	{
		while (SIM800LSerial.available()) DEBUG_PORT.write(SIM800LSerial.read());
		while (DEBUG_PORT.available()) SIM800LSerial.write(DEBUG_PORT.read());
	}
	*/
	flushSIM800Buffer();

	setup_MPU6050();
	pinMode(MPU6050_INTERRUPT_PIN, INPUT_PULLUP);
	writeByte(MPU6050_ADDRESS, INT_ENABLE, (1 << WOM_ENABLE));
	APP_STATE = SLEEP;
}

void loop() 
{
	//gpsPowerUp();
	//while(SIM800LSerial.available()) DEBUG_PORT.write(SIM800LSerial.read());
	//while (DEBUG_PORT.available()) SIM800LSerial.write(DEBUG_PORT.read());
	
	if (APP_STATE == SLEEP)
	{
		DEBUG_PORT.println(F("update in sleep mode"));
		positionUpdatesCounter = 1;
		schedulePositionUpdate = 0;
		motionTriggersCount = 0;
		gps_latitude = 0;
		gps_longitude = 0;

		gprsInitialized = 0;
		vBatt = map(analogRead(VBAT_MEAS_PIN), 0, 1023, 0, 17100) / 1000.0;
		zAccel = getZaccel();

    wakeCounts++;
		sim800initGPRS();
		updateStatusLocation(gps_latitude, gps_longitude, zAccel, vBatt, APP_STATE);     
    if(wakeCounts>GSM_CELL_DATA_UPDATE_SLEEP_INTERVAL)
    {
        wakeCounts=0;
        requestandParseCellTowers();
        serializeCells();
        DEBUG_PORT.println(serializedCells);
        updateGSMCellData(zAccel, vBatt, APP_STATE, serializedCells);
    }    
    sim800deinitGPRS();   
 
    readByte(MPU6050_ADDRESS, INT_CLEAR);
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), MPU6050_ISR, FALLING);

    int resetMotionCnt=0;
    int gpsPowerDownDelay=0;
		//go to sleep
    DEBUG_PORT.println(F("SLEEP"));
		for (int i = 0; i < SLEEP_MODE_WDOG_INTERVAL_SEC/8; i++)
		{
			delay(50);
			LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
			delay(50);

      gpsPowerDownDelay++;
      if(gpsPowerDownDelay>2)
      {
        gpsPowerDown();
      }

      resetMotionCnt++;
      if(!motionTriggered && resetMotionCnt>3)
      {
        resetMotionCnt=0;
        motionTriggersCount = 0;
      }
			
			if (motionTriggered)
			{
				motionTriggered = 0;
				motionTriggersCount++;
				DEBUG_PORT.print(F("MOTION - "));
				DEBUG_PORT.println(motionTriggersCount);
				readByte(MPU6050_ADDRESS, INT_CLEAR);
			}	
			if (motionTriggersCount > 2)
			{
        detachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN));
				motionTriggersCount = 0;
				APP_STATE = GPS_ON;
				break;
			}			
		}
	}
	
	if (APP_STATE == GPS_ON)
	{
		gpsPowerUp();
		APP_STATE = WAIT_FOR_FIX;
		DEBUG_PORT.println(F("gps on"));
		delay(500);
	}
	
	if ((APP_STATE == WAIT_FOR_FIX) || (APP_STATE == GPS_COMM_ERR))
	{
		timer_wdog_cur = millis();
		if ((timer_wdog_cur - timer_wdog_sta) >= (WAKE_MODE_WDOG_INTERVAL_MS))
		{
			DEBUG_PORT.println(F("update in wake mode"));
			timer_wdog_sta = millis();
			if(!gprsInitialized) sim800initGPRS();
			vBatt = map(analogRead(VBAT_MEAS_PIN), 0, 1023, 0, 17100) / 1000.0;
      zAccel = getZaccel();
			updateStatusLocation(gps_latitude, gps_longitude, zAccel, vBatt, APP_STATE);
			sim800deinitGPRS();
		}
	}

	if ((APP_STATE == HAVE_FIX) && schedulePositionUpdate)
	{
		DEBUG_PORT.println(F("updating location"));
		schedulePositionUpdate = 0;
		zAccel = getZaccel();
		if (!gprsInitialized) sim800initGPRS();
		vBatt = map(analogRead(VBAT_MEAS_PIN), 0, 1023, 0, 17100) / 1000.0;
		updateStatusLocation(gps_latitude, gps_longitude, zAccel, vBatt, APP_STATE);
		
		//if we have movement, coutinue updating dont go to sleep after 50 updates
		if (motionTriggered)
		{
			motionTriggered = 0;
			readByte(MPU6050_ADDRESS, INT_CLEAR);
			positionUpdatesCounter = 0;
		}
		else
		{
			positionUpdatesCounter++;
		}
	}
	
	if ((APP_STATE == WAIT_FOR_FIX) || (APP_STATE == GPS_COMM_ERR) || (APP_STATE == HAVE_FIX))
	{
		timer_gps_cur = millis();
		if (timer_gps_cur - timer_gps_sta >= GPS_POLL_INTERVAL_MS)
		{
			DEBUG_PORT.print(F("GPS READING...")); DEBUG_PORT.println(gpsFixAttempts);
			timer_gps_sta = millis();
		
			unsigned long fix_age;
			int newData = 0;;

			newData = 0;
			for (unsigned long start = millis(); millis() - start < 2000;)
			{
				while (gpsPort.available())
				{
          char c=gpsPort.read();
					gps.encode(c);
          DEBUG_PORT.print(c);
          
				}
			}
			
			if (gps.charsProcessed() < 10)
			{
				APP_STATE = GPS_COMM_ERR;
				gpsFixAttempts++;
			}
			else
			{
				if (gps.location.isValid())
				{
					APP_STATE = HAVE_FIX;
					gps_latitude =  gps.location.lat();
					gps_longitude = gps.location.lng();
					//WE HAVE FIX
					schedulePositionUpdate = 1;
				}
				else
				{
					//NO FIX
					gps_latitude = 0;
					gps_longitude = 0;
					APP_STATE = WAIT_FOR_FIX;
					gpsFixAttempts++;
				}
			}
		}
	}

	if (((APP_STATE == WAIT_FOR_FIX) || (APP_STATE == GPS_COMM_ERR)) && gpsFixAttempts>GPS_FIX_MAX_ATTEMPTS_BEFORE_SLEEP)
	{
		DEBUG_PORT.println(F("too many fix attempts"));
		gpsFixAttempts = 0;
		APP_STATE = SLEEP;
    wakeCounts=0;
    requestandParseCellTowers();
    serializeCells();
    //DEBUG_PORT.println(serializedCells);
    vBatt = map(analogRead(VBAT_MEAS_PIN), 0, 1023, 0, 17100) / 1000.0;
    zAccel = getZaccel();
    updateGSMCellData(zAccel, vBatt, APP_STATE, serializedCells);
		sim800deinitGPRS();
	}

	if ((APP_STATE == HAVE_FIX) && positionUpdatesCounter> GPS_MAX_POS_UPDATES_BEFORE_SLEEP)
	{
		DEBUG_PORT.println(F("finshed updating"));
		positionUpdatesCounter = 0;
		APP_STATE = SLEEP;
		sim800deinitGPRS();
	}
}
