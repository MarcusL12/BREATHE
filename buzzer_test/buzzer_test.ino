#define NOTE_Ab NOTE_Gs


// use 12 bit precision for LEDC timer
#define LEDC_TIMER_12_BIT 12

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN 33

#define ONVALUE 200

#define quarter_note_length 300

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(pin, duty);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  // Setup timer with given frequency, resolution and attach it to a led pin with auto-selected channel
  ledcAttach(LED_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);

}
void restNote(int time) {
  analogWrite(LED_PIN, 0);
  delay(time);
  analogWrite(LED_PIN, ONVALUE);
}

void playNote(note_t note, int timeLength, int octave) {
  // play the note for 90% of the time then turn off for a little
  ledcWriteNote(LED_PIN, note, octave);
  delay(timeLength * 0.9);
  restNote(timeLength * 0.1);
}

void playNote_legato(note_t note, int timeLength, int octave) {
  ledcWriteNote(LED_PIN, note, octave);
  delay(timeLength);
}

void loop() {
  // put your main code here, to run repeatedly:
  ledcAnalogWrite(LED_PIN, 200);
  restNote(quarter_note_length / 2);
  // measure 1/2
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 4);
  }
  playNote(NOTE_Eb, quarter_note_length * 3, 4);
  // measure 3-5
  restNote(quarter_note_length / 2);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_F, quarter_note_length / 2, 4);
  }
  playNote(NOTE_D, quarter_note_length * 3, 4);
  
  // measure 6
  restNote(quarter_note_length / 2);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 4);
  }
  // measure 7
  playNote(NOTE_Eb, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Ab, quarter_note_length / 2, 4);
  }
  //measure 8
  playNote(NOTE_G, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Eb, quarter_note_length / 2, 5);
  }
  // measure 9
  playNote_legato(NOTE_C, quarter_note_length * 2, 5);
  //measure 10
  playNote(NOTE_C, quarter_note_length / 2, 5);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 4);
  }

  //measure 11
  playNote(NOTE_D, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Ab, quarter_note_length / 2, 4);
  }

  //measure 12
  playNote(NOTE_G, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_F, quarter_note_length / 2, 5);
  }

  //measure 13
  playNote(NOTE_D, quarter_note_length * 2, 5);
  //measure 14
  playNote(NOTE_D, quarter_note_length / 2, 5);
  for (int i = 0; i < 2; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 5);
  }
  playNote(NOTE_F, quarter_note_length / 2, 5);

  //measure 15
  playNote(NOTE_Eb, quarter_note_length * 2, 5);

  //measure 16
  playNote(NOTE_D, quarter_note_length / 2, 5);
  for (int i = 0; i < 2; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 5);
  }
  playNote(NOTE_F, quarter_note_length / 2, 5);

  //measure 17
  playNote(NOTE_Eb, quarter_note_length * 2, 5);

  //measure 18
  playNote(NOTE_D, quarter_note_length / 2, 5);
  for (int i = 0; i < 2; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 5);
  }
  playNote(NOTE_F, quarter_note_length / 2, 5);

  //measure 19
  playNote(NOTE_Eb, quarter_note_length, 5);
  restNote(quarter_note_length);

  //measure 20
  playNote(NOTE_C, quarter_note_length, 5);
  restNote(quarter_note_length);

  //measure 21
  playNote(NOTE_G, quarter_note_length * 3, 5);

  //measure 22
  restNote(quarter_note_length / 2);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Ab, quarter_note_length / 2, 5);
  }

  //measure 23
  playNote(NOTE_F, quarter_note_length * 6, 5);

  restNote(5000);
  // for (int i = 0; i <= 1000; i = i + 100) {
  //   ledcChangeFrequency(LED_PIN, i, 12); // 1 khz, 12 bit resolution
  //   Serial.println(ledcReadFreq(LED_PIN));
  //   delay(100);
  // }
  // for (int i = 1000; i >= 0; i = i - 100) {
  //   ledcChangeFrequency(LED_PIN, i, 12); // 1 khz, 12 bit resolution
  //   Serial.println(ledcReadFreq(LED_PIN));
  //   delay(100);
  // }
  //ledcWriteTone(LED_PIN, 1000);
  //ledcWriteNote(LED_PIN, NOTE_C, 5);
  // analogWrite(LED_PIN, 200);
  //analogWriteFrequency(18, 1000);
}
