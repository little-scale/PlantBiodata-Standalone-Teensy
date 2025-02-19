#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=180,286
AudioAmplifier           amp1;           //xy=314,287
AudioEffectFade          fade1;          //xy=456,285
AudioEffectFreeverb      freeverb1;      //xy=576,324
AudioMixer4              mixer1;         //xy=741,303
AudioOutputMQS           mqs1;           //xy=1034,302
AudioConnection          patchCord1(playSdWav1, 0, amp1, 0);
AudioConnection          patchCord2(amp1, fade1);
AudioConnection          patchCord3(fade1, 0, mixer1, 0);
AudioConnection          patchCord4(fade1, freeverb1);
AudioConnection          patchCord5(freeverb1, 0, mixer1, 1);
AudioConnection          patchCord6(mixer1, 0, mqs1, 0);
AudioConnection          patchCord7(mixer1, 0, mqs1, 1);
// GUItool: end automatically generated code






#define SDCARD_CS_PIN BUILTIN_SDCARD
#define SDCARD_MOSI_PIN 11  // not actually used
#define SDCARD_SCK_PIN 13   // not actually used

const int interruptPin = 5;
const int LED1 = 2;
const int LED2 = 3;

const int MIDI_offset_pitch = 60;

const byte samplesize = 10;             //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array,

volatile unsigned long microseconds;  //sampling timer
volatile byte sampleIndex = 0;
volatile unsigned long samples[samplesize];

float threshold = 0.1;  //threshold multiplier
int threshold_data;
float threshMin = 1.61;  //scaling threshold min
float threshMax = 4.01;  //scaling threshold max
float prevThreshold = 0;

int noteMin = 36;         //C2  - keyboard note minimum
int noteMax = 96;         //C7  - keyboard note maximum
byte controlNumber = 80;  //set to mappable control, low values may interfere with other soft synth controls!!

byte rawSerial = 1;
unsigned long rawSerialTime = 0;
int rawSerialDelay = 0;

unsigned long currentMillis = 0;
unsigned long prevMillis = 0;

int setnote_previous;
int fadeTime = 25; 



void setup() {
  AudioMemory(128);
  Serial.begin(57600);
  pinMode(interruptPin, INPUT_PULLUP);  //pulse input
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  delay(250);

  digitalWrite(LED1, LOW);

  delay(250);
  digitalWrite(LED2, LOW);

  attachInterrupt(interruptPin, sample, RISING);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);
  amp1.gain(0.5); 

  freeverb1.roomsize(0.9); 
  freeverb1.damping(0.3); 

  delay(500);

  playSdWav1.stop();
  playSdWav1.play("01.wav");
  Serial.println("Playing SD Card Wave File");

  delay(500);
}

void loop() {
  currentMillis = millis();
  if (sampleIndex >= samplesize) {
    analyzeSample();
  }
  threshold = map((float)analogRead(A10), 0.0, 1023.0, 0.1, 4.0);
}

//*******Analysis of sample data
/*
 * Fill an array with samples
 * calcluate max, min, delta, average, standard deviation
 * compare against a 'change threshold' delta > (stdevi * threshold)
 * when change is detected map note pitch, velocity, duration
 * set the sample array size, the effective 'resolution' of change detection
 * using micros() for sample values, which is irratic, use a better method
 * sampling at the 'rising' edge, which counts a full period
 */
//*******


void sample() {
  if (sampleIndex < samplesize) {
    samples[sampleIndex] = micros() - microseconds;
    microseconds = samples[sampleIndex] + microseconds;  //rebuild micros() value w/o recalling
    //micros() is very slow
    //try a higher precision counter
    //samples[index] = ((timer0_overflow_count << 8) + TCNT0) - microseconds;
    sampleIndex += 1;
  }
}

void analyzeSample() {
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;
  int dur = 0;
  byte ccValue = 0;
  int ramp = 0;
  byte vel = 0;

  if (sampleIndex >= samplesize) {          //array is full
    unsigned long sampanalysis[analysize];  //copy to new array - is this needed?
    int setnote = 0;
    for (byte i = 0; i < analysize; i++) {
      //skip first element in the array
      sampanalysis[i] = samples[i + 1];  //load analysis table (due to volitle)
      //manual calculation
      if (sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if (sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];  //prep stdevi
    }

    //calculation
    averg = averg / analysize;
    stdevi = sqrt(stdevi / analysize - averg * averg);  //calculate stdevu
    if (stdevi < 1) { stdevi = 1.0; }                   //min stdevi of 1
    delta = maxim - minim;

    //**********perform change detection
    if (delta > (stdevi * threshold)) {
      change = 1;
    }
    //*********

    if (change) {
      

      //analyze the values
      dur = 150 + (map(delta % 127, 0, 127, 100, 5500));  //length of note
      ccValue = delta % 127;
      ramp = 3 + (dur % 100);  //control slide rate, min 25 (or 3 ;)
      vel = 90;                // this value should modulate
      vel = delta % 127;
      vel = map(vel, 0, 127, 80, 110);  //musical velocity range


      //set scaling, root key, note
      setnote = map(averg % 127, 0, 127, 1, 16);  //derive note, min and max note
      
      if (setnote != setnote_previous) {
        Serial.println(threshold);
        usbMIDI.sendNoteOff(setnote_previous + MIDI_offset_pitch, 0, 1);
        delay(10); 
        usbMIDI.sendNoteOn(setnote + MIDI_offset_pitch, 127, 1);

        setnote_previous = setnote; 

        Serial.println(setnote);
        fade1.fadeOut(fadeTime); 
        delay(fadeTime); 
        playSdWav1.stop();

        fade1.fadeIn(fadeTime); 

        switch (setnote) {
          case 1:
            playSdWav1.play("Sample_01.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 2:
            playSdWav1.play("Sample_02.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 3:
            playSdWav1.play("Sample_03.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 4:
            playSdWav1.play("Sample_04.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 5:
            playSdWav1.play("Sample_05.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 6:
            playSdWav1.play("Sample_06.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 7:
            playSdWav1.play("Sample_07.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 8:
            playSdWav1.play("Sample_10.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 9:
            playSdWav1.play("Sample_09.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 10:
            playSdWav1.play("Sample_010.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 11:
            playSdWav1.play("Sample_011.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 12:
            playSdWav1.play("Sample_012.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 13:
            playSdWav1.play("Sample_013.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 14:
            playSdWav1.play("Sample_014.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 15:
            playSdWav1.play("Sample_015.wav");
            Serial.println("Playing SD Card Wave File");
            break;

          case 16:
            playSdWav1.play("Sample_016.wav");
            Serial.println("Playing SD Card Wave File");
            break;
        }
      }

      // setnote = scaleNote(setnote, scaleSelect, root);  //scale the note
      // setnote = setnote + root; // (apply root?)
      // setNote(setnote, vel, dur, channel); //modify velocity, using note repetition or something?

      //derive control parameters and set
      // setControl(controlNumber, controlMessage.value, ccValue, ramp); //set the ramp rate for the control
    }

    if (rawSerial && (change || (currentMillis > rawSerialTime + rawSerialDelay))) {
      rawSerialTime = currentMillis;  //reset timer


      // Serial.print(map(constrain(threshold*stdevi,0,300),0,300,0,100)); Serial.print(","); //threshold compare against delta
      //  Serial.print(map(delta,0,300,0,100)); Serial.print(","); //delta
      //  Serial.print(change*90); //Serial.print(","); //change detected
      // Serial.println(); //end of raw data packet
    }

    sampleIndex = 0;
  }
}
