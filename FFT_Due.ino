/*

	Example of use of the FFT libray to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <Adafruit_NeoPixel.h>
#include "arduinoFFT.h"

//#define DEBUG_SETTINGS

#define FFT_SELECTED                          0
#define VOLUME_SELECTED                       1

#define DEFAULT_DISPLAY                       FFT_SELECTED

#define GREEN_COLOR                           strip1.Color(0,255,0)
#define YELLOW_COLOR                          strip1.Color(0,255,255)
#define RED_COLOR                             strip1.Color(255,0,0)
#define BLANK                                 strip1.Color(0,0,0)
     
#define POTENTIOMETER1                        A1
#define POTENTIOMETER2                        A4
#define POTENTIOMETER3                        A8
#define POTENTIOMETER4                        A2
#define POTENTIOMETER5                        A5
#define POTENTIOMETER6                        A9
#define POTENTIOMETER7                        A3
#define POTENTIOMETER8                        A6
#define POTENTIOMETER9                        A10


//#define POTENTIOMETER_LOW_GROUP               POTENTIOMETER3
//#define POTENTIOMETER_MID_GROUP               POTENTIOMETER6

#define POTENTIOMETER_LOW_LIMIT_SCALER        POTENTIOMETER7
#define POTENTIOMETER_MID_LIMIT_SCALER        POTENTIOMETER4
#define POTENTIOMETER_HIGH_LIMIT_SCALER       POTENTIOMETER1
#define POTENTIOMETER_CUTOFF_LOW              POTENTIOMETER8
#define POTENTIOMETER_CUTOFF_MID              POTENTIOMETER5
#define POTENTIOMETER_CUTOFF_HIGH             POTENTIOMETER2
#define POTENTIOMETER_LOW_BRIGHTNESS          POTENTIOMETER9
#define POTENTIOMETER_MID_BRIGHTNESS          POTENTIOMETER6
#define POTENTIOMETER_HIGH_BRIGHTNESS         POTENTIOMETER3

//#define POTENTIOMETER_SAMPLING_FREQUENCY      POTENTIOMETER2

//#define POTENTIOMETER_LIMIT_SCALER            POTENTIOMETER3

//#define POTENTIOMETER_VOLUME_LIMIT_SCALER     POTENTIOMETER3

#define USE_FILTER
//#define USE_BETTER_FILTER    
#define FILTER_DEPTH                          1

#define LIMIT_SCALER_BASE                     1

#define CHANNEL                               A0
#define SAMPLES                               64 //This value MUST ALWAYS be a power of 2
#define SAMPLING_FREQUENCY_BASE               3000
#define MIN_SAMPLING_FREQ                     500

#define MICROPHONE_OFFSET                     0

#define NUMBER_LEDS                           150

#define NUMBER_SAMPLES_PER_GROUP              round(NUMBER_LEDS/ (SAMPLES/2))

#define LOW_GROUP_BASE                        (2/16.0)
#define MID_GROUP_BASE                        (5/16.0)

#define BRIGHTNESS_SCALER                     1
#define CUTOFF_LIMIT_LOW_BASE                 0.07       //0.5
#define CUTOFF_LIMIT_MID_BASE                 0.07     //0.15 //0.3
#define CUTOFF_LIMIT_HIGH_BASE                0.07      //0.3

#define FFT_MAG_LIMIT_LOWS_BASE               7500//5000
#define FFT_MAG_LIMIT_MIDS_BASE               7500//20000 (PIANO) (256 samples)
#define FFT_MAG_LIMIT_HIGHS_BASE              7500//22000 (PIANO)
#define FFT_MAG_LIMIT_MIN                     350

#define BRIGHTNESS_LOW_BASE                   1
#define BRIGHTNESS_MID_BASE                   1
#define BRIGHTNESS_HIGH_BASE                  1

#define FFT_MAG_LIMIT_VOLUME_BASE             1000

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUMBER_LEDS, 2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUMBER_LEDS, 3, NEO_GRB + NEO_KHZ800);
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

float samplingFrequency = SAMPLING_FREQUENCY_BASE;  //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us = round(1000000*(1.0/samplingFrequency));
unsigned long microseconds;

float cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE;
float cutoffLimitMid = CUTOFF_LIMIT_MID_BASE;
float cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE;

float lowLimit = FFT_MAG_LIMIT_LOWS_BASE;
float midLimit = FFT_MAG_LIMIT_MIDS_BASE;
float highLimit = FFT_MAG_LIMIT_HIGHS_BASE;

float brightnessLow = BRIGHTNESS_LOW_BASE;
float brightnessMid = BRIGHTNESS_MID_BASE;
float brightnessHigh = BRIGHTNESS_HIGH_BASE;

float lowGroup = LOW_GROUP_BASE;
float midGroup = MID_GROUP_BASE;

float limitScaler = LIMIT_SCALER_BASE;

float volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE;

bool selectedDisplay = DEFAULT_DISPLAY;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[SAMPLES];
double vRealFilt[FILTER_DEPTH+1][SAMPLES];
double vRealSmoother[SAMPLES];
double vImag[SAMPLES];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  Serial.begin(115200);
  Serial.println("Ready");
  strip1.begin();
  strip1.show(); // Initialize all pixels to 'off'
  strip2.begin();
  strip2.show(); // Initialize all pixels to 'off'
}

void loop()
{
  UpdatePotentiometers();

  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<SAMPLES; i++)
  {
      vReal[i] = analogRead(CHANNEL)-MICROPHONE_OFFSET;
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  /* Print the results of the sampling according to time */
  //Serial.println("Data:");
  //PrintVector(vReal, SAMPLES, SCL_TIME);
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */  
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); /* Compute magnitudes */


#ifdef USE_FILTER
  static int counter = 0;
  for(int i=0; i<SAMPLES/2; i++)
  {
    //Serial.println("Iterate Samples");

  #ifdef USE_BETTER_FILTER
      vRealFilt[counter][i] = vReal[i];
      double total=0;
      for(int j=0; j<FILTER_DEPTH;j++)
      {
        total += vRealFilt[j][i];
      }
      
      vRealFilt[FILTER_DEPTH][i] = total/FILTER_DEPTH;
      
      vRealSmoother[i] = (vRealSmoother[i] + (double)vRealFilt[FILTER_DEPTH][i]) / 2; //vRealFilt[FILTER_DEPTH][i];//
  #else
      vRealSmoother[i]= (vRealSmoother[i]*0.25 + vReal[i]*0.75);
      //vRealSmoother[i] = vReal[i];
  #endif
  }
  counter++;
  if(counter>=FILTER_DEPTH)
  {
    counter=0;
  }
 #ifdef PRINT_FFT
  PrintVector(vRealSmoother, (SAMPLES >> 1), SCL_FREQUENCY);
 #endif
 
  if(selectedDisplay == FFT_SELECTED)
    DisplayFFTResponse(vRealSmoother);
  else
    DisplayAmbientNoiseMeter(vRealSmoother);
  
#else
  if(selectedDisplay == FFT_SELECTED)
    DisplayFFTResponse(vReal);
  else
    DisplayAmbientNoiseMeter(vReal);
=
#endif

  //delay(1000);
}


void DisplayFFTResponse(double * values)
{ 
  float brightnessScaling = 0;
  int j=2, subSamples=0;
  uint32_t colorSelector = strip1.Color(0,255,0);
  for(int i = 0; i < NUMBER_LEDS; i++)
  {
    bool SET_LEDS = false;

    //Select color per subset of frequencies
    if(i<(int)(NUMBER_LEDS* lowGroup))
    {      

        brightnessScaling = ((values[j])/(lowLimit));
        
        if(values[j] > lowLimit/2)
        {
          float overLimitValue = (values[j]-lowLimit/2);
          brightnessScaling += (overLimitValue/lowLimit) *2;
        }
        
        if(brightnessScaling < cutoffLimitLow)
          brightnessScaling = 0;
       
        int brightness = 255*brightnessLow*brightnessScaling;
        if (brightness > 255)
        {
          brightness = 255;
        }
        else if (brightness < 0)
          brightness = 0;
        colorSelector = strip1.Color(brightness,0,0);
    }
    else if (i<(int)(NUMBER_LEDS * midGroup))
    {
        brightnessScaling = ((values[j])/(midLimit));
        
        if(values[j] > midLimit/2)
        {
          float overLimitValue = (values[j]-midLimit/2);
          brightnessScaling += (overLimitValue/midLimit) *2;
        }
        if(brightnessScaling < cutoffLimitMid)
          brightnessScaling = 0;  
          
        int brightness = 255*brightnessMid*brightnessScaling;
        if(brightness > 255)
          brightness = 255;     
        else if (brightness < 0)
          brightness = 0;
        colorSelector = strip1.Color(brightness,brightness,0);
    }
    else 
    {
        brightnessScaling = ((values[j])/(highLimit));
        if(values[j] > highLimit/2)
        {
          float overLimitValue = (values[j]-highLimit/2);
          brightnessScaling += (overLimitValue/highLimit) *2;
        }
        if(brightnessScaling < cutoffLimitHigh)
          brightnessScaling = 0;
        

        int brightness = 255*brightnessHigh*brightnessScaling;
        if(brightness > 255)
          brightness = 255;
          else if (brightness < 0)
          brightness = 0;
        colorSelector = strip1.Color(0,brightness,0);
    }
    
    strip1.setPixelColor((i), colorSelector);
    strip2.setPixelColor((i), colorSelector);
    
    subSamples++;
    if(subSamples>NUMBER_SAMPLES_PER_GROUP)
    {
      j++;
      if(j>SAMPLES)
        break;
      subSamples=0;
    }
  }
  
  //Output to the LEDs
  strip1.show();
  strip2.show();
  
  delay(1); /* Repeat after delay */
}

void DisplayAmbientNoiseMeter(double * values)
{
  double total=0, maxAmp = 0;
  double totalAmplitude=0;
  for (int i = 0; i < SAMPLES/2; i++)
  {
    //Approach 2
    if(values[i]>volumeLimit)
    {
      totalAmplitude ++;
    }
    else
    {
      totalAmplitude += (values[i]/volumeLimit);
    }

    //Approach 1
    if(i<(int)(NUMBER_LEDS* lowGroup))
    {
      if(values[i]>volumeLimit)
      {
        total++;
      }
    }
    else if(i<(int)(NUMBER_LEDS* midGroup))
    {
      if(values[i]>volumeLimit)
      {
        total++;
      }
    }
    else
    {
      if(values[i]>volumeLimit)
      {
        total++;
      }
    }
    if(values[i]>maxAmp)
    {
      maxAmp = values[i];
    }
  }

//  Serial.print("total: ");
//  Serial.println(total);
//  Serial.print("totalAmplitude: ");
//  Serial.println(totalAmplitude);
  
  
  total *= NUMBER_SAMPLES_PER_GROUP;
  totalAmplitude *= NUMBER_SAMPLES_PER_GROUP;

  //Plotter Output
  Serial.print(total);
  Serial.print(" ");
  Serial.println(totalAmplitude);
  
  total = totalAmplitude;
  
  uint32_t colorSelector = strip1.Color(0,255,0);
  for(int i=0; i<NUMBER_LEDS; i++)
  {
    if(i<(NUMBER_LEDS*(lowGroup)))
    {      
      colorSelector = strip1.Color(255*BRIGHTNESS_SCALER,0,0);                            //RED
    }
    else if (i<(NUMBER_LEDS)*midGroup)
    {
      colorSelector = strip1.Color(255*BRIGHTNESS_SCALER,255*BRIGHTNESS_SCALER,0);        //YELLOW
    }
    else 
      colorSelector = strip1.Color(0,255*BRIGHTNESS_SCALER,0);                            //GREEN

    if(i>total)
    {    
      colorSelector=BLANK;
    }
    
    strip1.setPixelColor(i, colorSelector);    
    strip2.setPixelColor(i, colorSelector);
  }
  strip1.show();
  strip2.show();
  //delay(5);
}

void UpdatePotentiometers(void)
{
    
  float controlInput1 = analogRead(POTENTIOMETER1);
  float controlInput2 = analogRead(POTENTIOMETER2);
  float controlInput3 = analogRead(POTENTIOMETER3);
  float controlInput4 = analogRead(POTENTIOMETER4);
  float controlInput5 = analogRead(POTENTIOMETER5);
  //Serial.println(controlInput5);
  float controlInput6 = analogRead(POTENTIOMETER6);
  float controlInput7 = analogRead(POTENTIOMETER7);
  float controlInput8 = analogRead(POTENTIOMETER8);
  float controlInput9 = analogRead(POTENTIOMETER9);

  //controlInput1 = (controlInput1 /1023.0);  //Control input 0-1
  //controlInput1 = (controlInput1 /512.0);  //Control input 0-2
  controlInput1 = (controlInput1 / 1023.0) * 5;  //Control input 0-5
  //controlInput1 = (controlInput1 / 102.30);  //Control input 0-10
  
  //controlInput2 = (controlInput2 /1023.0);  //Control input 0-1
  //controlInput2 = (controlInput2 /512.0);  //Control input 0-2
  controlInput2 = (controlInput2 / 1023.0) * 5;  //Control input 0-5
  //controlInput2 = (controlInput2 / 102.30);  //Control input 0-10
  
  //controlInput3 = (controlInput3 /1023.0);  //Control input 0-1
  //controlInput3 = (controlInput3 /512.0);  //Control input 0-2
  controlInput3 = (controlInput3 / 1023.0) * 5;  //Control input 0-5
  //controlInput3 = (controlInput3 / 102.30);  //Control input 0-10
  
  //controlInput4 = (controlInput4 /1023.0);  //Control input 0-1
  //controlInput4 = (controlInput4 /512.0);  //Control input 0-2
  controlInput4 = (controlInput4 / 1023.0) * 5;  //Control input 0-5
  //controlInput4 = (controlInput4 / 102.30);  //Control input 0-10
  
  //controlInput5 = (controlInput5 /1023.0);  //Control input 0-1
  //controlInput5 = (controlInput5 /512.0);  //Control input 0-2
  controlInput5 = (controlInput5 / 1023.0) * 5;  //Control input 0-5
  //controlInput5 = (controlInput5 / 102.30);  //Control input 0-10
  
  //controlInput6 = (controlInput6 /1023.0);  //Control input 0-1
  //controlInput6 = (controlInput6 /512.0);  //Control input 0-2
  controlInput6 = (controlInput6 / 1023.0) * 5;  //Control input 0-5
  //controlInput6 = (controlInput6 / 102.30);  //Control input 0-10
  
  //controlInput7 = (controlInput7 /1023.0);  //Control input 0-1
  //controlInput7 = (controlInput7 /512.0);  //Control input 0-2
  controlInput7 = (controlInput7 / 1023.0) * 5;  //Control input 0-5
  //controlInput7 = (controlInput7 / 102.30);  //Control input 0-10
  
  //controlInput8 = (controlInput8 /1023.0);  //Control input 0-1
  //controlInput8 = (controlInput8 /512.0);  //Control input 0-2
  controlInput8 = (controlInput8 / 1023.0) * 5;  //Control input 0-5
  //controlInput8 = (controlInput8 / 102.30);  //Control input 0-10
  
  //controlInput9 = (controlInput9 /1023.0);  //Control input 0-1
  //controlInput9 = (controlInput9 /512.0);  //Control input 0-2
  controlInput9 = (controlInput9 / 1023.0) * 5;  //Control input 0-5
  //controlInput9 = (controlInput9 / 102.30);  //Control input 0-10

//  if(controlInput1>1)
//  {
//    selectedDisplay = FFT_SELECTED;
//  }
//  else 
//  {
//    selectedDisplay = VOLUME_SELECTED;
//  }
  #ifdef POTENTIOMETER_LOW_BRIGHTNESS
    #if POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER1
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput1;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER2
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput2;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER3
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput3;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER4
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput4;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER5
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput5;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER6
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput6;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER7
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput7;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER8
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput8;
    #elif POTENTIOMETER_LOW_BRIGHTNESS == POTENTIOMETER9
      brightnessLow = BRIGHTNESS_LOW_BASE * controlInput9;
    #endif

    #ifdef DEBUG_SETTINGS
      Serial.print("brightnessLow: ");
      Serial.println(brightnessLow);
    #endif
      
  #endif
  
   #ifdef POTENTIOMETER_MID_BRIGHTNESS
    #if POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER1
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput1;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER2
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput2;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER3
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput3;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER4
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput4;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER5
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput5;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER6
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput6;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER7
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput7;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER8
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput8;
    #elif POTENTIOMETER_MID_BRIGHTNESS == POTENTIOMETER9
      brightnessMid = BRIGHTNESS_MID_BASE * controlInput9;
    #endif

    #ifdef DEBUG_SETTINGS
      Serial.print("brightnessMid: ");
      Serial.println(brightnessMid);
    #endif
      
  #endif 
   
   #ifdef POTENTIOMETER_HIGH_BRIGHTNESS
    #if POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER1
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput1;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER2
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput2;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER3
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput3;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER4
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput4;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER5
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput5;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER6
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput6;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER7
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput7;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER8
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput8;
    #elif POTENTIOMETER_HIGH_BRIGHTNESS == POTENTIOMETER9
      brightnessHigh = BRIGHTNESS_HIGH_BASE * controlInput9;
    #endif

    #ifdef DEBUG_SETTINGS
      Serial.print("brightnessHigh: ");
      Serial.println(brightnessHigh);
    #endif
      
  #endif

 
  #ifdef POTENTIOMETER_LOW_GROUP  
  
    #if POTENTIOMETER_LOW_GROUP == POTENTIOMETER1
      lowGroup = LOW_GROUP_BASE * controlInput1;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER2
      lowGroup = LOW_GROUP_BASE * controlInput2;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER3
      lowGroup = LOW_GROUP_BASE * controlInput3;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER4
      lowGroup = LOW_GROUP_BASE * controlInput4;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER5
      lowGroup = LOW_GROUP_BASE * controlInput5;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER6
      lowGroup = LOW_GROUP_BASE * controlInput6;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER7
      lowGroup = LOW_GROUP_BASE * controlInput7;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER8
      lowGroup = LOW_GROUP_BASE * controlInput8;
    #elif POTENTIOMETER_LOW_GROUP == POTENTIOMETER9
      lowGroup = LOW_GROUP_BASE * controlInput9;
    #endif

    #ifdef DEBUG_SETTINGS
    Serial.print("lowGroup: ");
    Serial.println(lowGroup);
    #endif
    
  #endif
  
  #ifdef POTENTIOMETER_MID_GROUP  
  
    #if POTENTIOMETER_MID_GROUP == POTENTIOMETER1
      midGroup = MID_GROUP_BASE * controlInput1;
    #elif POTENTIOMETER_MID_GROUP == POTENTIOMETER2
      midGroup = MID_GROUP_BASE * controlInput2;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER3
      midGroup = MID_GROUP_BASE * controlInput3;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER4
      midGroup = MID_GROUP_BASE * controlInput4;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER5
      midGroup = MID_GROUP_BASE * controlInput5;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER6
      midGroup = MID_GROUP_BASE * controlInput6;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER7
      midGroup = MID_GROUP_BASE * controlInput7;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER8
      midGroup = MID_GROUP_BASE * controlInput8;
    #elif  POTENTIOMETER_MID_GROUP == POTENTIOMETER9
      midGroup = MID_GROUP_BASE * controlInput9;
    #endif
    
    #ifdef DEBUG_SETTINGS
    Serial.print("midGroup: ");
    Serial.println(midGroup);
    #endif
    
  #endif

  #ifdef POTENTIOMETER_SAMPLING_FREQUENCY
  
    #if POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER1
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput1;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER2
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput2;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER3
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput3;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER4
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput4;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER5
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput5;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER6
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput6;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER7
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput7;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER8
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput8;
    #elif POTENTIOMETER_SAMPLING_FREQUENCY == POTENTIOMETER9
      samplingFrequency = SAMPLING_FREQUENCY_BASE * controlInput9;
    #endif
    
    if(samplingFrequency < MIN_SAMPLING_FREQ)
    {
      samplingFrequency = MIN_SAMPLING_FREQ;
    }
    sampling_period_us = round(1000000*(1.0/samplingFrequency));
    
    #ifdef DEBUG_SETTINGS
    Serial.print("samplingFrequency: ");
    Serial.println(samplingFrequency);
    #endif
  #endif
  
  #ifdef POTENTIOMETER_CUTOFF_LOW

    #if POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER1
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput1;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER2
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput2;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER3
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput3;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER4
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput4;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER5
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput5;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER6
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput6;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER7
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput7;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER8
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput8;
    #elif POTENTIOMETER_CUTOFF_LOW == POTENTIOMETER9
      cutoffLimitLow = CUTOFF_LIMIT_LOW_BASE * controlInput9;
    #endif
    
    #ifdef DEBUG_SETTINGS
    Serial.print("cutoffLimitLow: ");
    Serial.println(cutoffLimitLow);
    #endif
  #endif

  #ifdef POTENTIOMETER_CUTOFF_MID

    #if POTENTIOMETER_CUTOFF_MID == POTENTIOMETER1
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput1;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER2
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput2;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER3
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput3;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER4
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput4;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER5
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput5;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER6
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput6;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER7
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput7;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER8
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput8;
    #elif POTENTIOMETER_CUTOFF_MID == POTENTIOMETER9
      cutoffLimitMid = CUTOFF_LIMIT_MID_BASE * controlInput9;
    #endif
    
    #ifdef DEBUG_SETTINGS
    Serial.print("cuffoffLimitMid: ");
    Serial.println(cutoffLimitMid);
    #endif 
    
  #endif
  
  #ifdef POTENTIOMETER_CUTOFF_HIGH

    #if POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER1
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput1;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER2
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput2;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER3
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput3;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER4
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput4;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER5
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput5;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER6
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput6;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER7
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput7;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER8
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput8;
    #elif POTENTIOMETER_CUTOFF_HIGH == POTENTIOMETER9
      cutoffLimitHigh = CUTOFF_LIMIT_HIGH_BASE * controlInput9;
    #endif
    
    #ifdef DEBUG_SETTINGS
    Serial.print("cutoffLimitHigh: ");
    Serial.println(cutoffLimitHigh);
    #endif
  #endif
  
  #ifdef POTENTIOMETER_LIMIT_SCALER

    #if POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER1
      limitScaler = LIMIT_SCALER_BASE * controlInput1;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER2
      limitScaler = LIMIT_SCALER_BASE * controlInput2;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER3
      limitScaler = LIMIT_SCALER_BASE * controlInput3;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER4
      limitScaler = LIMIT_SCALER_BASE * controlInput4;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER5
      limitScaler = LIMIT_SCALER_BASE * controlInput5;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER6
      limitScaler = LIMIT_SCALER_BASE * controlInput6;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER7
      limitScaler = LIMIT_SCALER_BASE * controlInput7;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER8
      limitScaler = LIMIT_SCALER_BASE * controlInput8;
    #elif POTENTIOMETER_LIMIT_SCALER == POTENTIOMETER9
      limitScaler = LIMIT_SCALER_BASE * controlInput9;
    #endif
    
    #ifdef DEBUG_SETTINGS
    Serial.print("limitScaler: ");
    Serial.println(limitScaler);
    #endif
  #endif
  
  #ifdef POTENTIOMETER_LOW_LIMIT_SCALER

    #if POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER1
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput1;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER2
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput2;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER3
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput3;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER4
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput4;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER5
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput5;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER6
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput6;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER7
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput7;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER8
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput8;
    #elif POTENTIOMETER_LOW_LIMIT_SCALER == POTENTIOMETER9
      lowLimit = FFT_MAG_LIMIT_LOWS_BASE * controlInput9;
    #endif

    if(lowLimit < FFT_MAG_LIMIT_MIN)
      lowLimit = FFT_MAG_LIMIT_MIN;
      
    #ifdef DEBUG_SETTINGS
    Serial.print("lowLimit: ");
    Serial.println(lowLimit);
    #endif
  #endif  
  
  #ifdef POTENTIOMETER_MID_LIMIT_SCALER

    #if POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER1
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput1;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER2
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput2;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER3
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput3;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER4
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput4;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER5
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput5;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER6
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput6;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER7
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput7;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER8
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput8;
    #elif POTENTIOMETER_MID_LIMIT_SCALER == POTENTIOMETER9
      midLimit = FFT_MAG_LIMIT_MIDS_BASE * controlInput9;
    #endif

    if(midLimit < FFT_MAG_LIMIT_MIN)
    {
      midLimit = FFT_MAG_LIMIT_MIN;
    }
    
    #ifdef DEBUG_SETTINGS
    Serial.print("midLimit: ");
    Serial.println(midLimit);
    #endif
  #endif
  
  #ifdef POTENTIOMETER_HIGH_LIMIT_SCALER

    #if POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER1
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput1;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER2
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput2;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER3
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput3;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER4
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput4;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER5
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput5;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER6
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput6;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER7
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput7;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER8
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput8;
    #elif POTENTIOMETER_HIGH_LIMIT_SCALER == POTENTIOMETER9
      highLimit = FFT_MAG_LIMIT_HIGHS_BASE * controlInput9;
    #endif
    
    if(highLimit < FFT_MAG_LIMIT_MIN)
    {
      highLimit = FFT_MAG_LIMIT_MIN;
    }
    
    #ifdef DEBUG_SETTINGS
    Serial.print("highLimit: ");
    Serial.println(highLimit);
    #endif
  #endif
  
  #ifdef POTENTIOMETER_VOLUME_LIMIT_SCALER

    #if POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER1
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput1;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER2
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput2;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER3
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput3;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER4
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput4;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER5
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput5;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER6
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput6;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER7
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput7;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER8
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput8;
    #elif POTENTIOMETER_VOLUME_LIMIT_SCALER == POTENTIOMETER9
      volumeLimit = FFT_MAG_LIMIT_VOLUME_BASE * controlInput8;
    #endif
    
    #ifdef DEBUG_SETTINGS
    Serial.print("volumeLimit: ");
    Serial.println(volumeLimit);
    #endif
  #endif
  
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 2; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / SAMPLES);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
