#include <EEPROM.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <ContainedNum.h>
#include <Encoder.h>
#include <Servo.h>
#include <MicroView.h>
#include <AD8495.h>

//pindef
//for WS2812 on the rotary encoder
#define datPin A4
#define clkPin A5
//rotary encoder
#define APin 2
#define BPin 3
//encoder button
#define pinDefBtnPin 5
//relay
#define outPin 6

//PID init
#define defaultSet 80
double pidIn=0;
double pidOut=0;
double pidSet=defaultSet;
double kp=40;
double ki=1;
double kd=1;
PID pid(&pidIn,&pidOut,&pidSet,kp,ki,kd,P_ON_M,DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;


//ATune init
double aTuneStep=500;
double aTuneNoise=1;
int aTuneLookBack=20;
PID_ATune atune(&pidIn,&pidOut);

//overall var
ContainedNum<char> mode(0,2);//PID,setting,autotune
ContainedNum<int> deg(-30,500);//set temperature
bool pidMode=0;//off, auto
 
 
// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}
 
// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
const int KpAddress = 0;
const int KiAddress = 8;
const int KdAddress = 16;
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, kp);
   }
   if (ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, ki);
   }
   if (kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, kd);
   }
}
 
// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   kp = EEPROM_readDouble(KpAddress);
   ki = EEPROM_readDouble(KiAddress);
   kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid

   if (isnan(kp))kp = 40;
   if (isnan(ki))ki = 0.5;
   if (isnan(kd))kd = 0.1;
}
//encwrap code
class EncWrap{
private:
  Encoder* WrappedEncoder;
  long lastState=0;
public:
  EncWrap(int pin1,int pin2);
  int getChg();
  long getPos();
};
EncWrap::EncWrap(int pin1,int pin2){
  WrappedEncoder=new Encoder(pin1,pin2);
}
int EncWrap::getChg(){
  int lastpos=lastState/4+(lastState%4)/2;//rounding division by 4
  long currState=WrappedEncoder->read();
  int currpos=currState/4+(currState%4)/2;
  lastState=currState;
  return currpos-lastpos;
}
long EncWrap::getPos(){
  long currState=WrappedEncoder->read();
  long currpos=currState/4+(currState%4)/2;
  return currpos;
}


//buttonpoll code
class Button{
  private:
  bool checkLong;
  bool down;
  unsigned long downAt; //timestamp of last push
  int btnPin;
  int lastType;
  int lastEvent;
  public:
  Button(int pin,bool longp){
    btnPin=pin;
    downAt=0;
    lastType=-2;
    lastEvent=-2;
    checkLong=longp;
  }
  Button(int pin):Button(pin,0){
    checkLong=0;
  }
  int pollBtn(){
    bool btnval=digitalRead(btnPin);
    int ret=-2;
    if(down==0&&btnval==1){
      //button push
      downAt=millis();
      down=1;
      ret=0;
    }
    else if(down==1){
      //button being pressed
      ret=-1;
      unsigned long currtime=millis();
      unsigned long dur=currtime-downAt;
      if(btnval==0){
        down=0;
        ret=1;//button release
      }
      if(dur>500)ret = 2;//long press
      //overrides release
    }
    if(lastType!=2){
      lastType=ret;
      lastEvent=ret;
    }else{
      lastType=ret;
    }
    return ret;
  }
  int getEvent(){
    int ret=lastEvent;
    lastEvent=-3;
    return ret;
  }
};

//inputs
AD8495 tc(2);
EncWrap enc(BPin,APin);
Button btn(pinDefBtnPin);

//pad number to 3 digits with spaces
void dispPadTmp(int t,int numFont){
  uView.setFontType(numFont);
  if(t>-100&&t<=-10){
    uView.print("-");
    t=-t;
  }
  else if(t<0){
    uView.print("- ");
    t=-t;
  }
  else if(t<10) uView.print("  ");
  else if(t<100) uView.print(" ");
  uView.print(t);
}
void dispPadTmp(double t,int numFont){
  uView.setFontType(numFont);
  if(t>-100&&t<=-10){
    uView.print("-");
    t=-t;
  }
  else if(t<0){
    uView.print("- ");
    t=-t;
  }
  else if(t==0){
    uView.print("  0.0");
    return;
  }
  else if(t<10) uView.print("  ");
  else if(t<100) uView.print(" ");
  //keep 1 dp
  long t10=t*10;
  long tint=t10/10;
  int tdec=t10%10;
  uView.print(tint);
  uView.print(".");
  uView.print(tdec);
}
//display degC or degF symbol
void dispDeg(char CF){
  uView.setFontType(0);
  uView.print("o");
  uView.setFontType(1);
  uView.print(CF);
}

//PID ui
int lastTemp=0;
unsigned long lastUpdate=0;
void dispTemp(int curr,int set,int perc,int sec,bool on,bool inATune=false,unsigned int ATuneTime=0){
  uView.clear(PAGE);
  uView.setColor(WHITE);
  
  uView.setCursor(0,2);
  uView.setFontType(0);
  uView.print("Cur:");
  uView.setCursor(24,0);

  unsigned long now=millis();
  if(now-lastUpdate>500){
    lastTemp=curr;
    lastUpdate=now;
  }
  dispPadTmp(lastTemp,1);
  dispDeg('C');
  
  if(inATune){
    int atSec=ATuneTime%60;
    int atMin=ATuneTime/60%60;
    int atHrs=ATuneTime/3600;
    uView.setCursor(0,17);
    uView.setFontType(0);
    uView.print("ATunSp:");
    dispPadTmp(set,0);
    if(atHrs<10)uView.print("0");
    uView.print(atHrs);
    uView.print(":");
    if(atMin<10)uView.print("0");
    uView.print(atMin);
    uView.print(":");
    if(atSec<10)uView.print("0");
    uView.print(atSec);
  }
  else if(pidMode==0){
    uView.setFontType(0);
    uView.setCursor(0,21);
    uView.print("MANUAL");
  }
  else{
    uView.setFontType(0);
    uView.setCursor(0,21);
    uView.print("Set:");
    uView.setCursor(24,19);
    dispPadTmp(set,1);
    dispDeg('C');
  }
  
  uView.setCursor(0,38);
  dispPadTmp(perc,0);
  uView.print("%");

  uView.setCursor(27,38);
  dispPadTmp(sec,0);
  uView.print("s");

  if(on) uView.circleFill(58,41,5);
  else uView.circle(58,41,5);
  uView.display();
}

unsigned long atuneStartTime=0;
int atuneSet=80;
bool atuneStarted=false;
void pidUI(int chg,int btnEvt,unsigned long now,bool on,bool inAtune=false){
  if(!inAtune){
    if(pidMode){
      //automatic mode, change setpoint
      deg+=chg;
      pidSet=(double)int(deg);
    }
    else{
      //manual mode, change percentage
      int perc=100.0*pidOut/(double)WindowSize;
      perc+=chg;
      if(perc>100)perc=100;
      else if(perc<0)perc=0;
      pidOut=(double)perc*(double)WindowSize/100.0;
    }

    if(btnEvt==1){
      //change pid mode on press
      pidMode=!pidMode;
      windowStartTime = now;
    }
    else if(btnEvt==2)mode=1;//enter setting on long press
    
  }
  else{
    if(btnEvt==2){
      mode=1;
      atuneStarted=0;
      atune.Cancel();
    }
    else if(atuneStarted){
      if(atune.Runtime()){
        kp=(double)round(atune.GetKp()*10)/10;
        ki=(double)round(atune.GetKi()*10)/10;
        kd=(double)round(atune.GetKd()*10)/10;
        mode=1;
        atuneStarted=0;
      }
    }
    else{
      pidMode=0;
      atuneStarted=1;
      atuneSet=pidIn;
      atuneStartTime=now;
      windowStartTime=now;
      atune.Runtime();
    }
  }
  int secs=(WindowSize-(now-windowStartTime))/1000;
  int perc=pidOut/WindowSize*100;
  if(inAtune)dispTemp(pidIn,atuneSet,perc,secs,on,true,(now-atuneStartTime)/1000);
  else dispTemp(pidIn,deg,perc,secs,on);
}


//display setting screen
//item 0 Back
//item 1 Atune
//item 2 Kp
//item 3 Ki
//item 4 Kd
//item 5 reset
ContainedNum<char> setSelItem(0,5);
bool setSel=0;
ContainedNum<char> chgMag(1,100);//1,10,100
void dispSetting(int item,bool entered){
  uView.clear(PAGE);
  uView.setColor(WHITE);
  uView.setCursor(0,0);
  uView.setFontType(0);

  uView.print("Back\n");
  uView.print("Autotune\n");
  uView.print("Kp:");
  dispPadTmp(kp,0);
  uView.print("\nKi:");
  dispPadTmp(ki,0);
  uView.print("\nKd:");
  dispPadTmp(kd,0);

  uView.setCursor(0,40);
  if(setSel){
    char plusminus=240;
    uView.print(plusminus);
    dispPadTmp((double)chgMag/10,0);
  }
  else{
    uView.print("Reset");
  }
  int rectX=0;
  int rectY=item*8;
  int rectWid=64,rectHei=8;
  if(entered)uView.rect(rectX,rectY,rectWid,rectHei);
  else uView.rectFill(rectX,rectY,rectWid,rectHei,WHITE,XOR);
  uView.display();
}


void setUI(int chg,int btnEvt){
  //handle state chage
  if(setSel){
    //modifying pid parameters
    switch(setSelItem){
      case 2:
      kp=(double)(round(kp*10)+chg*chgMag)/10;
      break;
      case 3:
      ki=(double)(round(ki*10)+chg*chgMag)/10;
      break;
      case 4:
      kd=(double)(round(kd*10)+chg*chgMag)/10;
    }
    if(btnEvt==1)setSel=!setSel;//exit modify mode on press
    else if(btnEvt==2){
      //change magnitude on long press
      if(chgMag==1)chgMag=10;
      else if(chgMag==10)chgMag=100;
      else chgMag=1;
    }
  }
  else{
    //selecting menu item
    setSelItem-=chg;
    if(btnEvt==1){
      switch(setSelItem){
        case 0:
        mode--;
        pid.SetTunings(kp,ki,kd);
        SaveParameters();
        break;
        case 1:
        mode++;
        break;
        case 2:case 3:case 4:
        setSel=!setSel;
        break;
        case 5:
        kp=65;
        ki=0.5;
        kd=40;
        SaveParameters();
      }
    }
  }
  dispSetting(setSelItem,setSel);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  uView.begin();
  uView.clear(PAGE);
  pinMode(datPin,OUTPUT);
  pinMode(clkPin,OUTPUT);
  pinMode(APin,INPUT);
  pinMode(BPin,INPUT);
  pinMode(pinDefBtnPin,INPUT);
  pinMode(outPin,OUTPUT);
  LoadParameters();
  mode=0;
  pidMode=0;
  deg=defaultSet;
  pid.SetOutputLimits(0, WindowSize);
  pid.SetSampleTime(WindowSize);
  pid.SetTunings(kp,ki,kd);
  atune.SetOutputStep(aTuneStep);
  atune.SetControlType(1);
  atune.SetLookbackSec(aTuneLookBack);
  atune.SetNoiseBand(aTuneNoise);
  tc.init();
}

void loop() {
  // put your main code here, to run repeatedly:
	//read sensor
  tc.poll();
  if(millis()%100<5)pidIn=tc.tempC();

  //run pid
  pid.SetMode(pidMode);
  bool pidComputed=pid.Compute();//sync time window
  
  //drive relay output
  unsigned long now = millis();
  bool on;

  //determine when to update window
  if(pidComputed)windowStartTime=now;//if PID updates output reset window
  else if( (mode==2||pidMode==0) && (now - windowStartTime>WindowSize) )windowStartTime += WindowSize;//if autotune or manual shift window

  //determine on off state of relay
  if(pidOut > now - windowStartTime)on=true;
  else on=false;
  digitalWrite(outPin,on);
  
  //poll user input
  btn.pollBtn();
  int btnEvt=btn.getEvent();
  int chg=enc.getChg();
  
  //display UI
  switch(mode){
    case 0:
    pidUI(chg,btnEvt,now,on);
    break;
    case 1:
    setUI(chg,btnEvt);
    break;
    case 2:
    pidUI(chg,btnEvt,now,on,true);
  }
}
