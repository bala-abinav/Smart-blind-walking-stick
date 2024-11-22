// Ultrasonic Sensor Pins
const int trigPin = 9;
const int echoPin = 10;
const int obstacleBuzzer = 11;  

// Pulse Sensor and Heart Rate Buzzer Pins
int pulsePin = A0;
int blinkPin = 13;
int heartBuzzer = 12;

// Emergency Push Button Pin
const int buttonPin = 2;

// Variables for Ultrasonic Sensor
long duration;
int distance;
int safetyDistance = 50;

// Variables for Pulse Sensor
volatile int BPM;
volatile int Signal;
volatile int IBI = 600;
volatile boolean Pulse = false;
volatile boolean QS = false;

volatile int rate[10];
volatile unsigned long sampleCounter = 0;   
volatile unsigned long lastBeatTime = 0;    
volatile int P = 512;             
volatile int T = 512;             
volatile int thresh = 525;        
volatile int amp = 100;           
volatile boolean firstBeat = true;
volatile boolean secondBeat = false;

// Heart rate threshold settings
const int maxHeartRate = 120;
const int minHeartRate = 60;

// Define normal heart rate limit
const int normalHeartRateLimit = 100;

// Safety time settings
const unsigned long safetyTime = 30000;
unsigned long abnormalStartTime = 0;
bool isAbnormal = false;

void setup() {
  // Setup for Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(obstacleBuzzer, OUTPUT);

  // Setup for Pulse Sensor
  pinMode(blinkPin, OUTPUT);
  pinMode(heartBuzzer, OUTPUT);

  // Setup for Emergency Push Button
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(115200);
  interruptSetup();
}

void loop() {
  checkUltrasonicSensor();
  checkEmergencyButton();
  checkHeartRateAlert();
  delay(20);

  // Print the heart rate continuously
  if (QS) {  
    Serial.print("Current Heart Rate: ");
    Serial.println(BPM);
    QS = false;
  }

  // Handle abnormal heart rate condition
  if (BPM > maxHeartRate || BPM < minHeartRate) {
    if (!isAbnormal) {
      isAbnormal = true;
      abnormalStartTime = millis();
    } else if (millis() - abnormalStartTime >= safetyTime) {
      digitalWrite(heartBuzzer, HIGH);
    }
  } else {
    isAbnormal = false;
    digitalWrite(heartBuzzer, LOW);
  }
}

// Function to check the ultrasonic sensor for obstacle detection
void checkUltrasonicSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  if (distance <= safetyDistance) {
    digitalWrite(obstacleBuzzer, HIGH);
  } else {
    digitalWrite(obstacleBuzzer, LOW);
  }
}

// Function to check the emergency push button state
void checkEmergencyButton() {
  if (digitalRead(buttonPin) == LOW) {
    digitalWrite(heartBuzzer, HIGH);
  }
}

// Function to check if heart rate exceeds threshold
void checkHeartRateAlert() {
  if (QS == true) {
    serialOutputWhenBeatHappens();

    // Check if BPM is above or below limits
    if (BPM > maxHeartRate) {
      Serial.println("Heart Rate: Too High!");
    } else if (BPM < minHeartRate) {
      Serial.println("Heart Rate: Too Low!");
    } else {
      Serial.println("Heart Rate: Normal");
    }
  }
}

// Pulse Sensor Interrupt Setup
void interruptSetup() {
  TCCR2A = 0x02;
  TCCR2B = 0x06;
  OCR2A = 0X7C;
  TIMSK2 = 0x02;
  sei();
}

// Output BPM to Serial Monitor
void serialOutputWhenBeatHappens() {
  Serial.print("Heart-Beat Found. BPM: ");
  Serial.println(BPM);
}

// Timer interrupt to read pulse sensor data every 2ms
ISR(TIMER2_COMPA_vect) {
  cli();
  Signal = analogRead(pulsePin);
  sampleCounter += 2;
  int N = sampleCounter - lastBeatTime;

  if (Signal < thresh && N > (IBI / 5) * 3) {
    if (Signal < T) T = Signal;
  }
  if (Signal > thresh && Signal > P) P = Signal;

  if (N > 250 && (Signal > thresh) && !Pulse && N > (IBI / 5) * 3) {
    Pulse = true;
    digitalWrite(blinkPin, HIGH);
    IBI = sampleCounter - lastBeatTime;
    lastBeatTime = sampleCounter;

    if (secondBeat) {
      secondBeat = false;
      for (int i = 0; i <= 9; i++) rate[i] = IBI;
    }

    if (firstBeat) {
      firstBeat = false;
      secondBeat = true;
      sei();
      return;
    }

    word runningTotal = 0;
    for (int i = 0; i <= 8; i++) {
      rate[i] = rate[i + 1];
      runningTotal += rate[i];
    }
    rate[9] = IBI;
    runningTotal += rate[9];
    runningTotal /= 10;
    BPM = 60000 / runningTotal;
    QS = true;
  }

  if (Signal < thresh && Pulse) {
    digitalWrite(blinkPin, LOW);
    Pulse = false;
    amp = P - T;
    thresh = amp / 2 + T;
    P = thresh;
    T = thresh;
  }

  if (N > 2500) {
    thresh = 512;
    P = 512;
    T = 512;
    lastBeatTime = sampleCounter;
    firstBeat = true;
    secondBeat = false;
  }
  sei();
}
