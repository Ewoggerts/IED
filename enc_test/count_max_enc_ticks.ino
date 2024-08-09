// Define the pin for the encoder signal
const int encoderPin = 2;

// Variables to track the ticks and time
volatile unsigned long ticks = 0;
unsigned long tickCounts[10] = {0};
volatile bool secondElapsed = false;
int secondCounter = 0;

// Interrupt service routine to count ticks
void encoderISR() {
  ticks++;
}

// Timer interrupt service routine
ISR(TIMER1_COMPA_vect) {
  secondElapsed = true;
}

void setupTimer() {
  noInterrupts();  // Disable all interrupts

  // Configure timer1 for 1Hz interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 15624;  // Compare match register for 1Hz increments (16MHz/1024/1Hz - 1)
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt

  interrupts();  // Enable all interrupts
}

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Set the encoder pin as input
  pinMode(encoderPin, INPUT);
  
  // Attach interrupt to the encoder pin
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
  
  // Setup the timer interrupt
  setupTimer();

  delay(2000);
}

void loop() {
  if (secondElapsed) {
    // Store the tick count for the current second
    tickCounts[secondCounter] = ticks;
    
    // Reset the tick count for the next second
    ticks = 0;
    
    // Increment the second counter
    secondCounter++;
    
    // Check if 10 seconds have elapsed
    if (secondCounter >= 10) {
      // Calculate the average ticks per second
      unsigned long totalTicks = 0;
      for (int i = 0; i < 10; i++) {
        totalTicks += tickCounts[i];
      }
      unsigned long averageTicksPerSecond = totalTicks / 10;
      
      // Print the average ticks per second over the last 10 seconds
      Serial.print("Average ticks per second over the last 10 seconds: ");
      Serial.println(averageTicksPerSecond);
      
      // Reset for the next 10-second period
      secondCounter = 0;
    }
    
    // Reset the flag
    secondElapsed = false;
  }
}
