#include <Math.h>
#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>
#include <Servo.h>
#include <Wire.h>
#include <WireExt.h>
//read from csv files eachline is 20/30ms
//Vibrate.java form stream.start
//look for encoding into PWR[i] in andriod
//create a buffer for 7 bytes
//send one character instead of
//copy Disable() similar to sending

#define UPDATE_PERIOD      5

#define MAX_MOTOR_LIST_SIZE      100
#define TOTAL_MOTOR                3

#define MSG_ENABLE         '1'
#define MSG_DISABLE        '2'
#define MSG_PWR_DATA       '3'
#define MSG_CLR_DATA       '4'
#define MSG_PWR_DATA_OK    '5'
#define MSG_RCV_SUCCESS    '6'
#define MSG_PWR_DATA_FULL  '7'
#define MSG_PWR_DATA_END   '8'
#define HEAD_HAPPY_SQZ     'a'
#define HEAD_SAD           'b'
#define HEAD_AFRAID        'c'
#define HEAD_SURPRISED     'd'
#define HEAD_EX_HAPPY      'f'
#define HEAD_HAPPY_STR     'g'
#define TEMPERATURE        'e'
#define HEAD_NEUTRAL       'h'

#define D6T_addr 0x0A
#define D6T_cmd 0x4C

//Declaration of I2C port nos for Thermopile
Servo topdownservo;
Servo leftrightservo;

struct MotPin
{
	int iPwr;
        int iPos;
};

struct MotControl
{
	bool bDisable;
	int iMaxPos;
	int iMinPos;
};

#define PIN_ENABLE 22

MotPin pPin[2];
MotControl pCtrl[2];

int i,pMode=0;
 int iMot;
//Current power for motor to move---> current mode it is in
int pPwr[2];
//Timeout to update data
unsigned long ulMillis;

//Current power for motor to move

//List of motor values
int lVal[MAX_MOTOR_LIST_SIZE][TOTAL_MOTOR];
int iValHeadIdx;
int iValTailIdx;
int iValSize;
int iValTimeout;

char pBuf[30];
int iBuf;
char pMsg[6+3+TOTAL_MOTOR*2];

//Declaration for Thermopile
int rbuf[35];
float tdata[16];
float t_PTAT;
float Avg_Temp;

AndroidAccessory pAndroidUSB("Google, Inc.", "DemoKit", "DemoKit Arduino Board", "1.0", "http://www.android.com", "0000000012345678");

void setup()
{
	//Delay parameters
	ulMillis = millis();

	memset(pPwr, 0, sizeof(pPwr));

	//Init motor parameters
	pPin[0].iPwr = 9;pPin[0].iPos = 0;
        pPin[1].iPwr = 10;pPin[1].iPos = 0;

	pCtrl[0].bDisable = false; pCtrl[0].iMaxPos = 180; pCtrl[0].iMinPos = 0;
	pCtrl[1].bDisable = false; pCtrl[1].iMaxPos = 180; pCtrl[1].iMinPos = 0;
	MotPin* pMotPin;
	for (int i=0; i<2; i++)
	{
		pMotPin = &pPin[i];
		pinMode(pMotPin->iPwr, OUTPUT);
	}
	topdownservo.attach(pPin[0].iPwr);
        leftrightservo.attach(pPin[1].iPwr);

	//List of values
	iValHeadIdx = 0;
	iValTailIdx = 0;
	iValSize = 0;
	iValTimeout = 0;

	iBuf = 0;
	memset(pBuf, sizeof(pBuf), 0);
	pMsg[0] = 0; pMsg[1] = 0; pMsg[2] = 0;

	DisableMotor();
	Serial.begin(115200);
	Serial.print("BEGIN");
	pAndroidUSB.powerOn();

        Wire.begin();
}

void ReadADK()
{
	//Within a very short time frame, we will do following to ensure fast response
	// - Read command from host
	// - Response to host to let host know the command result (such as memory is
	//   full, or receive successfully)
	// - Exit this function. Because motor needs critical time, any pending command
	//   will need to wait for next loop. Host will definitely wait for reply from
	//   Arduino to send more command. Otherwise, it will make serial port congest

	//Read at most 40 characters into buffer
	int iRdLen = pAndroidUSB.read(&pBuf[iBuf], sizeof(pBuf)-iBuf, 1);
	if (iRdLen > 0)
	{
		iBuf += iRdLen;
		ParseMsg();
	}
}

void ShiftBuf(int iIdx)
{
	iBuf -= iIdx;
	for (int i=0; i<iBuf; i++)
		pBuf[i] = pBuf[i+iIdx];
}

void ParseMsg()
{
	if (iBuf <= 4)
		return;

	int i;
	int iMsgLen;
	int iTmp;

	while (iBuf > 4)
	{
		//First 3 bytes must be 0
		if (pBuf[0] != 0)
		{
			ShiftBuf(1);
			continue;
		}
		if (pBuf[1] != 0)
		{
			ShiftBuf(2);
			continue;
		}
		if (pBuf[2] != 0)
		{
			ShiftBuf(3);
			continue;
		}

		//4th byte as message length
		iMsgLen = pBuf[3];
		if (iMsgLen == 0 || iMsgLen > 1+TOTAL_MOTOR*2)
		{
			ShiftBuf(4);
			return;
		}
		else if ((7 + iMsgLen) > iBuf)
			return;

		//Last 3 byte must be 1
		if (pBuf[4+iMsgLen] != 1 || pBuf[5+iMsgLen] != 1 || pBuf[6+iMsgLen] != 1)
		{
			ShiftBuf(4);
			continue;
		}
		ShiftBuf(4);
		ParseCmd(iMsgLen, pBuf);
		ShiftBuf(iMsgLen+3);
	}
}

void ParseCmd(int iMsgLen, char* pMsg)
{
	char cChar;
        Serial.println(pMsg[0]);
	switch (pMsg[0])
	{
		case MSG_PWR_DATA:
		{
			if (iMsgLen != 1+TOTAL_MOTOR*2)
				return;
			EnableMotor();

			if (iValSize < MAX_MOTOR_LIST_SIZE)
			{
				for (int i=0; i<TOTAL_MOTOR; i++)
				{
					lVal[iValTailIdx][i] = (int)(pMsg[1+i*2] & 0xFF);
					if ((pMsg[2+i*2] & 0xFF) == 0xFF)
						lVal[iValTailIdx][i] = -lVal[iValTailIdx][i];
				}
				cChar = MSG_PWR_DATA_OK;
				iValSize++;
				iValTailIdx = (iValTailIdx+1)%MAX_MOTOR_LIST_SIZE;

				//If memory is full, inform Android to pause sending immediately
				if (iValSize == MAX_MOTOR_LIST_SIZE)
				{
					cChar = MSG_PWR_DATA_FULL;
					Serial.println("DataMemFull");
				}
			}
			else
			{
				cChar = MSG_PWR_DATA_FULL;
				Serial.println("DataMemFull");
			}
			ResponseShort(cChar);
		}
		break;
		case MSG_CLR_DATA:
			Serial.println("Clear data");
			iValSize = 0;
			iValTailIdx = 0;
			iValHeadIdx = 0;
			for (int i=0; i<2; i++)
				//SetPwr(i, 0);
			ResponseShort(MSG_RCV_SUCCESS);
		break;
		case MSG_ENABLE:
			Serial.println("Enable motor");
			EnableMotor();
			ResponseShort(MSG_RCV_SUCCESS);
		break;
		case MSG_DISABLE:
			Serial.println("Disable motor");
			DisableMotor();
			ResponseShort(MSG_RCV_SUCCESS);
		break;
                case HEAD_HAPPY:
                       Serial.println("HAPPY");
                       HappyShake(2);
                       ResponseShort(MSG_RCV_SUCCESS);
                       break;
                case HEAD_SAD:
                        Serial.println("SAD");
                       ShakeHead(1,3);
                       ResponseShort(MSG_RCV_SUCCESS);
                       break;
                case HEAD_AFRAID:
                       Serial.println("AFRAID");
                        leftrightservo.write(120);        
                        ShakeHead(0,1);
                       ResponseShort(MSG_RCV_SUCCESS);
                       break;
                case HEAD_SURPRISED:
                       Serial.println("SURPRISED");
                       leftrightservo.write(120);        
                       ShakeHead(0,3);
                       ResponseShort(MSG_RCV_SUCCESS);
                       break;
}
}

void ResponseShort(char cCmd)
{
	pMsg[3] = 1;
	pMsg[4] = cCmd;
	pMsg[5] = 1; pMsg[6] = 1; pMsg[7] = 1;
	pAndroidUSB.write(&pMsg[0], 8);
}

void EnableMotor()
{
	for (int i=0; i<2; i++)
        pPin[i].iPos=50;
}

void DisableMotor()
{
	for (int i=0; i<2; i++)
        pPin[i].iPos=pPin[i].iPos;
}
void HappyShake(int iSpeed)
{
   for(pPin[1].iPos =pPin[1].iPos; pPin[1].iPos < 180; pPin[1].iPos += iSpeed)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    leftrightservo.write(pPin[1].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  
  for(pPin[0].iPos = 180; pPin[0].iPos >= 1; pPin[0].iPos -= iSpeed)     // goes from 180 degrees to q0 degrees 
  {                                
    topdownservo.write(pPin[0].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
  for(pPin[0].iPos= pPin[0].iPos; pPin[0].iPos < 180; pPin[0].iPos += iSpeed)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    topdownservo.write(pPin[0].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pPin[1].iPos = 180; pPin[1].iPos>=1; pPin[1].iPos -= iSpeed)     // goes from 180 degrees to 0 degrees 
  {                                
    leftrightservo.write(pPin[1].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
 for(pPin[0].iPos = 180; pPin[0].iPos >= 1; pPin[0].iPos -= iSpeed)     // goes from 180 degrees to q0 degrees 
  {                                
    topdownservo.write(pPin[0].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
  for(pPin[0].iPos= pPin[0].iPos; pPin[0].iPos < 180; pPin[0].iPos += iSpeed)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    topdownservo.write(pPin[0].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}
void ShakeHead(int iMot, int iSpeed)//0->Up 1->Side
{
  
  if(iMot==0)
 {
  for(pPin[0].iPos= pPin[0].iPos; pPin[0].iPos < 180; pPin[0].iPos += iSpeed)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    topdownservo.write(pPin[0].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pPin[0].iPos = 180; pPin[0].iPos >= 1; pPin[0].iPos -= iSpeed)     // goes from 180 degrees to q0 degrees 
  {                                
    topdownservo.write(pPin[0].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
}else if(iMot==1)
 {
  for(pPin[1].iPos =pPin[1].iPos; pPin[1].iPos < 180; pPin[1].iPos += iSpeed)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    leftrightservo.write(pPin[1].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pPin[1].iPos = 180; pPin[1].iPos>=1; pPin[1].iPos -= iSpeed)     // goes from 180 degrees to 0 degrees 
  {                                
    leftrightservo.write(pPin[1].iPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
 }

}
/*void SetPwr(int iMot, int iPwr)
{
	MotPin* pMotPin = &pPin[iMot];
	if (pCtrl[iMot].bDisable)
		return;

	if (iPwr < 0)
	{
		digitalWrite(pMotPin->iDin2, HIGH);
		digitalWrite(pMotPin->iDin1, LOW);
	}
	else
	{
		digitalWrite(pMotPin->iDin2, LOW);
		digitalWrite(pMotPin->iDin1, HIGH);
	}

	//Adjust the power to prevent motor move too fast or not enough power to move
	unsigned int uiAbsPwr = abs(iPwr);
	if (uiAbsPwr > pCtrl[iMot].iMaxPwr)
		uiAbsPwr = pCtrl[iMot].iMaxPwr;
	else if (uiAbsPwr < pCtrl[iMot].iMinPwr && uiAbsPwr != 0)
		uiAbsPwr = pCtrl[iMot].iMinPwr;
	analogWrite(pMotPin->iPwr, uiAbsPwr);
}
*/

void thermopile()
{
  int i;
  
    Wire.beginTransmission(D6T_addr);
    Wire.write(D6T_cmd);
    Wire.endTransmission();
    
      if (WireExt.beginReception(D6T_addr) >= 0)
      {
        i = 0;
        for (i = 0; i < 35; i++) {
          rbuf[i] = WireExt.get_byte();
        }
        WireExt.endReception();
 
        t_PTAT = (rbuf[0]+(rbuf[1]<<8))*0.1;
        for (i = 0; i < 16; i++) {
          tdata[i]=(rbuf[(i*2+2)]+(rbuf[(i*2+3)]<<8))*0.1;
        }
        
        Avg_Temp = (tdata[5] + tdata[6] + tdata[9] + tdata[10])/4;
        Serial.print("Average Temperature =");
        Serial.println(Avg_Temp); 
       }
         delay(500);
}

void Temperature()
{
  float Avg_Temp_Display;
  
        if(Avg_Temp<=30.0 && Avg_Temp>5.0) //Ambient temperature
        {

Avg_Temp_Display = Avg_Temp; 
        } 
        else if(Avg_Temp>30.0 && Avg_Temp<36.0)//Body temperature
        {
          Avg_Temp_Display = Avg_Temp * 1.08;
        }
        else if(Avg_Temp>=36.0 && Avg_Temp<40.0)//High temperature
        {
          Avg_Temp_Display = Avg_Temp;
        }
        else //No user
        {
          Serial.println("No user");
          Avg_Temp_Display = 0;
        }
        
        //Changing values to send
        int Avg_Temp_Int = Avg_Temp_Display;
        int Avg_Temp_Decimal = (Avg_Temp_Display * 10);
        Avg_Temp_Decimal = Avg_Temp_Decimal % 10;
        
        //Show edited values
        Serial.print(Avg_Temp_Int);
        Serial.print(".");
        Serial.println(Avg_Temp_Decimal);
        
        //Send temperature data to Android in int
        pMsg[0] = 0; pMsg[1] = 0; pMsg[2] = 0;
	pMsg[3] = 4;
        pMsg[4] = TEMPERATURE;
	pMsg[5] = Avg_Temp_Int/10;
        pMsg[6] = Avg_Temp_Int%10;
        pMsg[7] = Avg_Temp_Decimal;
	pMsg[8] = 1; pMsg[9] = 1; pMsg[10] = 1;
	pAndroidUSB.write(&pMsg[0], 11);
}

void loop()
{
	if (pAndroidUSB.isConnected())
	{
		int i;
		unsigned long ulNow = millis();


                thermopile();
                Temperature();
                
		if (ulNow > ulMillis + UPDATE_PERIOD)
		{
			//Update new power from Android
			if (iValSize > 0)
			{
				int iCntZero = 0;
				for (i=0; i<TOTAL_MOTOR; i++)
				{
					pPwr[i] = lVal[iValHeadIdx][i];
					if (pPwr[i] == 0)
						iCntZero++;
				}
				if (iCntZero == TOTAL_MOTOR)
				{
					ResponseShort(MSG_PWR_DATA_END);
					iValTimeout = 101;
					DisableMotor();
				}
				else
					iValTimeout = 0;
				if (iValSize <= MAX_MOTOR_LIST_SIZE)
					ResponseShort(MSG_PWR_DATA_OK);
				iValSize--;
				iValHeadIdx = (iValHeadIdx+1)%MAX_MOTOR_LIST_SIZE;
			}
			else if (iValTimeout < 100)
				iValTimeout++;
			else if (iValTimeout == 100)
			{
				iValTimeout++;
				ResponseShort(MSG_PWR_DATA_END);
				memset(pPwr, 0, sizeof(pPwr));
				DisableMotor();
			}

			//Update timer to next counter. Addition will guarantee the counting of
			//timer instead of the distance between 2 time spots
			ulMillis += UPDATE_PERIOD;

			//Set motor power
			//for (i=0; i<TOTAL_MOTOR; i++)
				//SetPwr(i, pPwr[i]);
		}

		//Monitor serial data (connecting to shield) to response on time
		ReadADK();
	}
	else
	{
		DisableMotor();
	}
}

