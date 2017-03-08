#include <Button.h>
#include <SD.h>
#include <SPI.h>
#define BUTTONPIN 11
#define SOLENOID 12
#define SAMPLESIZE 3700 //number of data points to record. Limited by RAM, each data point takes one 32-bit data point, so 4 bytes.
#define FILTERINGTIME 1200 //Time, in microseconds, to stagger the positive and negative averages for the bandpass filter
#define EXTRADELAY 38 //Extra delay every loop while logging, need to uncomment a line in the code also.
#define MICROPHONEGAIN 15 //The microphone gain can be adjusted by applying ground or voltage to one of its pins.


//Code for fast ADC function. This is all magic as far as I understand. From here - http://forum.arduino.cc/index.php?topic=338640.0
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

// DAC
static __inline__ void DACsync() __attribute__((always_inline, unused));
static void DACsync() {
  while (DAC->STATUS.bit.SYNCBUSY == 1);
}

//End magic

uint32_t SolenoidOpenUntil;
uint32_t SolenoidOpenTime;

uint32_t DataTimes[SAMPLESIZE]; //stores both the time and output. Output is stored in first 12 bits, time is stored in last 20 bits

const int SolenoidOpenDuration = 20; //How long to 
const int NextPulse = 1000;
const float SpeedOfSound = 1130.0; //speed of sound, in desired unit per second
int SolenoidOn = 0;
int LowGain = 1;

//For a simple bandpass filter, we'll average measurements over the previous time period, and subtract the average from the time period of the same length just prior to this time period, if that makes any sense.
int32_t RecentTimeSum;
int16_t RecentPoints;
int32_t PriorTimeSum;
int16_t PriorPoints;

uint32_t NextPeakFindingTime = 0;
int16_t PeakThreshold = 125; //Threshold for considering a peak
uint32_t PeakDeadband = 10000; //number of microseconds to ignore after a peak is detected
uint32_t PeakTimes[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //Time points where peaks were detected. At most, collect 20 peaks.
uint16_t PeakHeights[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //Maximum amplitude of the detected peaks
uint8_t NumberPeaks = 0;

const int16_t chipSelect = 4;
const int16_t LowGainTime = 35; //How many milliseconds to keep the microphone in low-gain mode after triggering the solenoid
const uint32_t LoggerInterval = 30000; //30 second logger interval

bool LoggingOn = false;

//If LogEverything is true, it will save the raw data from each run. Be warned, this is a monstrous amount of data.
const bool LogEverything = true;

uint32_t NextLoggerTime = 0;

Button button(BUTTONPIN);

void setup() {

  
  ADCsync();
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain select as 1X
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA

  
  // Set sample length and averaging
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
  ADCsync();
  ADC->SAMPCTRL.reg = 0x1F;  ; //sample length in 1/2 CLK_ADC cycles Default is 3F
  
  //Control B register
  int16_t ctrlb = 0x400;       // Control register B hibyte = prescale, lobyte is resolution and mode 
  ADCsync();
  ADC->CTRLB.reg =  ctrlb     ; 
  anaRead(A0);  //Discard first conversion after setup as ref changed


  //throw away first button output
  button.pressed();
  
  pinMode(MICROPHONEGAIN, OUTPUT);
  pinMode(SOLENOID, OUTPUT);



  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("Time (1/10000 sec),Piezo Reading");
  dataFile.close();

  NextPeakFindingTime = 2 * FILTERINGTIME;

  delay(1000);
}

void loop() {

  if (button.pressed()){
    //Toggle logging
    LoggingOn = 1 - LoggingOn;

    if (LoggingOn){
      digitalWrite(8, HIGH);
    } else {
      digitalWrite(8, LOW);
    }
    delay(100);
  }
  


  if (LoggingOn  && (SolenoidOpenUntil+NextPulse < millis()) && (millis() > NextLoggerTime )){
    //Time to do another tap
    NextLoggerTime = millis() + LoggerInterval;
    digitalWrite(MICROPHONEGAIN, HIGH);
    NextPeakFindingTime = 0;
    for (int i = 0; i < 20; i++){
      PeakTimes[i] = 0;
      PeakHeights[i] = 0;
    }
    NumberPeaks = 0;
    LowGain = 1;
    delay(1000); //wait 1 second before triggering solenoid

    SolenoidOn = 1;
    digitalWrite(SOLENOID, HIGH);
    SolenoidOpenTime = micros();
    
    //record X000 data points
    for (int DataIndex = 0; DataIndex <=SAMPLESIZE-1; DataIndex++){
      
      if ((micros() - SolenoidOpenTime > SolenoidOpenDuration * 1000) && (SolenoidOn == 1 )){
        //if the time for the solenoid to turn off is passed, turn it off
        digitalWrite(SOLENOID, LOW);
        SolenoidOn = 0;
      }
      if ((micros() - SolenoidOpenTime > LowGainTime * 1000 )&& (LowGain == 1)){
        //Put the microphone into high gain mode by setting the pin low, making it more sensitive to hear the echo
        digitalWrite(MICROPHONEGAIN, LOW);
        LowGain = 0;
      }
      //Cram both the time stamp and the reading into a single 32 bit integer variable by shifting the timestamp 12 bits and adding it to the data reading (which only goes up to 2^12)
      DataTimes[DataIndex] = anaRead(A0) | (uint32_t)((micros()-SolenoidOpenTime) << 12);
      //If you want to manually add extra delay to increase the time between samples, uncomment the line below
      //delayMicroseconds(EXTRADELAY);
    }
 

    //dump data
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    File RawdataFile = SD.open("RawData.txt",FILE_WRITE);
    
    dataFile.println();

    if (LogEverything){
      RawdataFile.print("Run time ");
      RawdataFile.println(NextLoggerTime-LoggerInterval);
    }

    delay(100);

    //Loop through all of the data points. It does weird stuff to the last few data points, so just drop them.
    for (int DataIndex = 0; DataIndex <=SAMPLESIZE-10; DataIndex++){
      
     
      if (LogEverything){  
        RawdataFile.print(getTime(DataTimes[DataIndex]));
        RawdataFile.print(",");
        RawdataFile.print(getData(DataTimes[DataIndex]));
      }

      //Now, do a poor-boy bandpass filter by taking the difference between the average readings 1100 us apart

      if (getTime(DataTimes[DataIndex]) > 2*FILTERINGTIME){
        PriorTimeSum = 0;
        RecentTimeSum = 0;
        RecentPoints = 0;
        PriorPoints = 0;
        for (int i = DataIndex; i > 0; i--){
          if (getTime(DataTimes[DataIndex]) - getTime(DataTimes[i]) < FILTERINGTIME){
            RecentTimeSum = RecentTimeSum + getData(DataTimes[i]);
            RecentPoints++;
          } else if (getTime(DataTimes[DataIndex]) - getTime(DataTimes[i]) < (2 * FILTERINGTIME)){
            PriorTimeSum = PriorTimeSum + getData(DataTimes[i]);
            PriorPoints++;
          } else {
            
            uint32_t FilteredVal = abs((int32_t)(RecentTimeSum /RecentPoints) - (int32_t)(PriorTimeSum / PriorPoints));

            if (LogEverything){
              RawdataFile.print(",");
              RawdataFile.print(FilteredVal);
            }

            //If the filtered value is greater than the threshold for what we call a "peak", then go here
            if (FilteredVal > PeakThreshold){

              //If we haven't found any values above the peak height for a time greater than the deadband period, then log it as the next peak
              if (getTime(DataTimes[DataIndex]) > NextPeakFindingTime){
                NumberPeaks++;
                PeakTimes[NumberPeaks] = getTime(DataTimes[DataIndex]);
              }

              //If this peak's value is greater than the current maximum observed height for this particular peak, make note of it. Note that we don't record the time at which the maximum height is observed, we're recording the rising edge instead.
              if (PeakHeights[NumberPeaks] < FilteredVal){
                PeakHeights[NumberPeaks] = FilteredVal;
              }

              //Note what time it is, and extend the deadband until we see a quiet period with no values above the threshold. 
              
              NextPeakFindingTime = PeakTimes[NumberPeaks] + PeakDeadband;
            }
            
            i = 0;
            
          }
        }
      }
        

        if (LogEverything){
          RawdataFile.println();
        }
    
      
    }

    if (LogEverything){
      RawdataFile.println();
      
    }
    RawdataFile.close();

    dataFile.print(NextLoggerTime-LoggerInterval);
    dataFile.print(" - ");
    dataFile.println("Peak Times");
    for (int i = 1; i < 20; i++){
      //record the times where peaks over the threshold were observed, along with their relative heights
      dataFile.print(PeakTimes[i]);
      dataFile.print(",");
      dataFile.println(PeakHeights[i]);
    }

    dataFile.close();

    //next, report the time between the first two highest peaks. This is most likely the depth.
    if (NumberPeaks > 1){
    
      File dataFile = SD.open("DepthLog.txt", FILE_WRITE);
      dataFile.println();
     
      dataFile.print(NextLoggerTime-LoggerInterval);
      
      dataFile.print(",");
      
      //We will assume the biggest peak after the first peak is the first echo. Find that loudest peak after the first peak.
      uint8_t BiggestPeak = 0;
      for (int i=2; i <= NumberPeaks; i++){

        if (PeakHeights[i] > PeakHeights[BiggestPeak]){
          BiggestPeak = i;
        }
        
      
      }
      //Calculate half-travel distance from echo time. Divide by 1 million to convert microseconds to seconds and divide by 2 to convert total sound travel distance to half-distance (depth)
      dataFile.print((PeakTimes[BiggestPeak]-PeakTimes[1])*SpeedOfSound/2000000.0);
      dataFile.close();
      
    }
    
  } else {
    delay(20);
  }


}

//Bitshift to cram the time and data into a single variable. Un-bitshift them to get them back out.
uint32_t getTime(uint32_t RawData){

  return RawData >> 12;
  
}

uint16_t getData(uint32_t RawData){

  return ((RawData << 20) >> 20);
  
}

//Fast analog read from http://forum.arduino.cc/index.php?topic=338640.0
uint32_t anaRead(uint32_t ulPin) {

  ADCsync();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input

  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared

  ADCsync();
  ADC->SWTRIG.bit.START = 1;                // Start ADC conversion

  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Wait till conversion done
  ADCsync();
  uint32_t valueRead = ADC->RESULT.reg;

  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable the ADC 
  ADCsync();
  ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
  return valueRead;
}
