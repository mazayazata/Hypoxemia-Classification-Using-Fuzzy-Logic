#include <Adafruit_GFX.h>        //OLED libraries
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
int hitung=0;

MAX30105 particleSensor;
#define SAMPLING 5 
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
int beatAvg1;
double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;

int i = 0;
int k;
int Num = 100;//calculate SpO2 by this sampling interval

double ESpO2 = 95.0;//initial value of estimated SpO2
double ESpO21;
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
String Status;

double SNormal;
double SMild;
double SModerate;
double SSevere;

double Hbrad;
double Hfine;
double Htac;

double predictNormal;
double predictMild;
double predictModerate;
double predictSevere;

double defuzzySugeno;

double PNormal;
double PMild;
double PModerate;
double PSevere;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,              //Logo2 and Logo3 are two bmp pictures that display on the OLED if called
0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,  };

static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00  };

#define USEFIFO
void setup() {  
  
  Serial.begin(115200); 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);
  // Initialize sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running

}

int n = 0;
//-------------------------------------------------------fuzzyfikasi SpO2----------------------------------------------------------------------------------------------------------------------
double SpO2Normal(double ESpO2) //nilai fuzzy hypoxemia normal 
{ 
  if(ESpO2 < 94)
  {
   return 0; 
  }
  else if((ESpO2 >= 94) && (ESpO2 < 95))
  {
    return (ESpO2 - 94)/(95-94);
  }
  else if((ESpO2 >= 95) && (ESpO2 <= 100))
  {
    return 1;
  }
}

double SpO2Mild(double ESpO2)//nilai fuzzy hypoxemia mild
{
  if(ESpO2 < 90)
  {
    return 0; 
  }
  else if((ESpO2 >= 90) && (ESpO2 < 91))
  {
    return (ESpO2 - 90)/(91-90);
  }
  else if((ESpO2 >= 91) && (ESpO2 <= 94))
  {
    return 1;
  }
  else if((ESpO2 > 94) && (ESpO2 <= 95))
  {
    return (95-ESpO2)/(95-94);
  }
  if(ESpO2 > 95)
  {
    return 0; 
  }
}

double SpO2moderate(double ESpO2){ //nilai fuzzy hypoxemia moderate
  if(ESpO2 < 85)
  {
    return 0; 
  }
  else if((ESpO2 >= 85) && (ESpO2 < 86))
  {
    return (ESpO2 - 85)/(86-85);
  }
  else if((86 <= ESpO2) && (ESpO2 <= 90))
  {
    return 1;
  }
  else if((ESpO2 > 90) && (ESpO2 <= 91))
  {
    return (91-ESpO2)/(91-90);
  }
  if(ESpO2 > 91)
  {
    return 0; 
  }
}

double SpO2severe(double ESpO2) //nilai fuzzy hypoxemia severe
{
  if(ESpO2 <= 85)
  {
    return 1; 
  }
  else if((ESpO2 > 85) && (ESpO2 <= 86))
  {
    return (86-ESpO2)/(86-85);
  }
  else if(ESpO2 > 86)
  {
    return 0;
  }
}
//-------------------------------------------------------fuzzyfikasi Heart Rate----------------------------------------------------------------------------------------------------------------

double HRbrad(double beatAvg) //nilai fuzzy heart rate bradhycardia
{
  if(beatAvg <= 60)
  {
    return 1; 
  }
  else if((beatAvg > 60) && (beatAvg <= 80))
  {
    return (80-beatAvg)/(80-60);
  }
  else if(beatAvg > 80)
  {
    return 0;
  }
}

double HRfine(double beatAvg) //nilai fuzzy heart rate normal
{
  if(beatAvg < 60)
  {
    return 0; 
  }
  else if((beatAvg >= 60) && (beatAvg <= 80))
  {
    return (beatAvg - 60)/(80-60);
  }
  else if((beatAvg > 80) && (beatAvg <= 100))
  {
    return (100 - beatAvg)/(100-80);
  }
  if(beatAvg > 100)
  {
    return 0; 
  }
}

double HRtac(double beatAvg) //nilai fuzzy heart rate tachycardia
{
  if(beatAvg > 100)
  {
    return 1; 
  }
  else if((beatAvg > 80) && (beatAvg <= 100))
  {
    return (beatAvg - 80)/(100-80);
  }
  else if(beatAvg <= 80)
  {
    return 0;
  }
}

//------------------------------------------------define minimum-------------------------------------------------------------------------------------------------------------------------------

double Min(double a, double b) {
  if (a < b) {
    return a;
  }
  else if (b < a) {
    return b;
  }
  else return a;
}

//----------------------------------------------define maximum---------------------------------------------------------------------------------------------------------------------------------

double Max2para(double a, double b) {
  if (a > b) {
    return a;
  }
  else if (b > a) {
    return b;
  }
  else return a;
}

double Max3para(double a, double b, double c) {
  if ((a > b) && (a > c)) {
    return a;
  }
  else if ((b > a) && (b > c)) {
    return b;
  }
  else if ((c > a) && (c > b)) {
    return c;
  }
  else if (a = b){
    return a;
  }
  else if (b = c){
    return b;
  }
  else if (a = c){
    return a;
  }
}

double Max4para(double a, double b, double c, double d) {
  if ((a > b) && (a > c) && (a > d)) {
    return a;
  }
  else if ((b > a) && (b > c) && (b > d)) {
    return b;
  }
  else if ((c > a) && (c > b) && (c > d)) {
    return c;
  }
  else if ((d > a) && (d > b) && (d > c)) {
    return d;
  }
  else if (a = b){
    return a;
  }
  else if (b = c){
    return b;
  }
  else if (a = d){
    return a;
  }
  else if (b = d){
    return b;
  }
  else if (c = d){
    return c;
  }
  else if (a = c){
    return a;
  }
}


//---------------------------------------------------inference---------------------------------------------------------------------------------------------------------------------------------

double infNormal(double a, double b, double c)
{
  a = SpO2Normal(ESpO2);
  b = HRbrad(beatAvg);
  c = HRfine(beatAvg);
  double normal_hypoxia1 = Min(a, b);
  double normal_hypoxia2 = Min(a, c);
  return Max2para(normal_hypoxia1, normal_hypoxia2);
}

double infMild(double a, double b, double c, double d, double e)
{ 
  a = SpO2Normal(ESpO2);
  b = HRtac(beatAvg);
  c = SpO2Mild(ESpO2);
  d = HRbrad(beatAvg);
  e = HRfine(beatAvg);
  double mild_hypoxia1 = Min(a, b);
  double mild_hypoxia2 = Min(c, d);
  double mild_hypoxia3 = Min(c, e);
  return Max3para(mild_hypoxia1, mild_hypoxia2, mild_hypoxia3);
}

double infModerate(double a, double b, double c, double d, double e)
{ 
  a = SpO2Mild(ESpO2);
  b = HRtac(beatAvg);
  c = SpO2moderate(ESpO2);
  d = HRbrad(beatAvg);
  e = HRfine(beatAvg);
  double moderate_hypoxia1 = Min(a, b);
  double moderate_hypoxia2 = Min(c, d);
  double moderate_hypoxia3 = Min(c, e);
  return Max3para(moderate_hypoxia1, moderate_hypoxia2, moderate_hypoxia3);
}

double infSevere(double a, double b, double c, double d, double e, double f)
{
  a = SpO2moderate(ESpO2);
  b = HRtac(beatAvg);
  c = SpO2severe(ESpO2);
  d = HRbrad(beatAvg);
  e = HRfine(beatAvg);
  f = HRtac(beatAvg);   
  double severe_hypoxia1 = Min(a, b);
  double severe_hypoxia2 = Min(c, d);
  double severe_hypoxia3 = Min(c, e);
  double severe_hypoxia4 = Min(c, f);
  return Max4para(severe_hypoxia1, severe_hypoxia2, severe_hypoxia3, severe_hypoxia4);
}

//----------------------------------------------------Defuzzyfikasi Sugeno---------------------------------------------------------------------------------------------------------------------

double Sugeno(double infNormal,double infMild,double infModerate,double infSevere)
{
  return ((infNormal*100)+(infMild*90)+(infModerate*80)+(infSevere*65))/(infNormal+infMild+infModerate+infSevere);
}

double SpO2 = 0; //raw SpO2 before low pass filtered

//-------------------------------------------------void----------------------------------------------------------------------------------------------------------------------------------------
void fuzzyfikasi(){
  SNormal = SpO2Normal(ESpO2);
  SMild = SpO2Mild(ESpO2);
  SModerate = SpO2moderate(ESpO2);
  SSevere = SpO2severe(ESpO2);
  
  Hbrad = HRbrad(beatAvg);
  Hfine = HRfine(beatAvg);
  Htac = HRtac(beatAvg);
  predictNormal = infNormal(SNormal, Hbrad, Hfine);
  predictMild = infMild(SNormal,Htac,SMild,Hbrad,Hfine);
  predictModerate = infModerate(SMild,Htac,SModerate,Hbrad,Hfine);
  predictSevere = infSevere(SModerate,Htac,SSevere,Hbrad,Hfine,Htac);
  
  defuzzySugeno = Sugeno(predictNormal,predictMild,predictModerate,predictSevere); 
      
}

//-------------------------------------------------main----------------------------------------------------------------------------------------------------------------------------------------
void loop() { 
  long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
  uint32_t ir, red , green;
  double fred, fir;

if(irValue > 7000){                                           //If a finger is detected
   #ifdef USEFIFO
     particleSensor.check(); //Check the sensor, read up to 3 samples
   #ifdef MAX30105
     red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
     ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
   #else
     red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
     ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
   #endif    
     i++;
     fred = (double)red;
     fir = (double)ir;
     avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
     aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
     sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
     sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
   #endif     
   if ((i % Num) == 0) {
     double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
     SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
     ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
     sumredrms = 0.0; sumirrms = 0.0; i = 0;
  }
  if (checkForBeat(irValue) == true)                       //If a heart beat is detected
  {  
    display.clearDisplay();                                   //Clear the display
//    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);       //Draw the first bmp picture (little heart)
    display.setTextSize(1);                                   //Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE); 
    display.setCursor(20,0);                
    display.println("Hasil Prediksi");             
    display.setCursor(20,18);
    display.println(k);             
    display.display();
    
    long delta = millis() - lastBeat;                   //Measure duration between two beats
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);           //Calculating the BPM

    if (beatsPerMinute < 255 && beatsPerMinute > 20)               //To calculate the average we strore some values (4) then do some math to calculate the average
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }

}
 
fuzzyfikasi();

if (ESpO2 > 100){
      Serial.println("Tidak Valid");
  }
  else{
       if (defuzzySugeno <= 75){
          k=4;
       }
       else if ((defuzzySugeno > 75)&&(defuzzySugeno < 85)){
          k=3;
       }
       else if ((defuzzySugeno >= 85)&&(defuzzySugeno < 95)){
          k=2;
       }
       else if (defuzzySugeno >= 95){
          k=1;
       }
  }   
    hitung++;
    if(hitung>=100){
      hitung=0;       
      Serial.print("#");
      Serial.print(beatAvg);
      Serial.print(",");
      Serial.print(ESpO2);
      Serial.print(",");
      Serial.print(k);
      Serial.println(" ");
//      Serial.println(defuzzySugeno);
   }
  
 
}

  
  if (irValue < 7000){       //If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
     ESpO2 = 0;
     beatAvg=0;
     fuzzyfikasi();
     k = 0;
//     hitung++;
//     if(hitung>=100){
//       hitung=0;       
//       Serial.print("#");
//       Serial.print(beatAvg);
//       Serial.print(",");
//       Serial.print(ESpO2);
//       Serial.print(",");
//       Serial.print(k);
//       Serial.println(" ");
//       Serial.println(defuzzySugeno);
//    }
     
     display.clearDisplay();
     display.setTextSize(1);                    
     display.setTextColor(WHITE);             
     display.setCursor(30,5);                
     display.println("Please Place "); 
     display.setCursor(30,15);
     display.println("your finger ");  
     display.display();
     noTone(3);
   }
}
