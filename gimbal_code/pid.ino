#define SAMPLE_TIME 100
#define SETPOINT 0
#define OUT_MIN 35
#define OUT_MAX 80

unsigned long lastTime;
double outputSum, lastInput;

void initPID(){
  lastTime = millis() - SAMPLE_TIME;
}

void computePID(){
  unsigned long currTime = millis();
  unsigned long dT = (currTime-lastTime);

  if(dT >= SAMPLE_TIME)
  {
    float error = SETPOINT - input;
    float dInput = input - lastInput;
    outputSum += (KI*error);

    if(outputSum > OUT_MAX) outputSum = OUT_MAX;
    else if(outputSum < OUT_MIN) outputSum = OUT_MIN;
    
    float output;
    output = (KP * error) + outputSum - KD * dInput;

    if(output > OUT_MAX) output = OUT_MAX;
    else if(output < OUT_MIN) output = OUT_MIN;
    Serial.println(output);
    lastInput = input;
    lastTime = currTime;
  }
}

