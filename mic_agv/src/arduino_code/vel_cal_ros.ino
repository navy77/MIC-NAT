const uint8_t DIR[] = {7, 42, 5, 38};
const uint8_t PWM[] = {6, 39, 4, 37};
const uint8_t EA[] = {10,36,13,33};
const uint8_t EB[] = {11,35,12,34};

long int enc_count[4];
long int enc_count_0;
long int enc_count_1;
long int enc_count_2;
long int enc_count_3;

long int prv_enc_count[4];

const int TPM[] = {902,1003};//tick/meter
long prv_time[2];
float distance[4];
float d_time;
float vel_act[4];
float vel_flt[4];
float vel_prv[4];
float vel_t[4];

int pwm[4];
int dir[4];
int enc_value[4];

// PID 
float prv_err[4];
float err[4];
float I_err[4];
float D_err[4];
float err_p[4];
float KP = 50;
float KI = 10;
float KD = 5;

void setup() {
  Serial.begin(115200);
    for(int i=0;i<4;i++){
      pinMode(EA[i],INPUT_PULLUP);
      pinMode(EB[i],INPUT_PULLUP);
      pinMode(PWM[i],OUTPUT);
      pinMode(DIR[i],OUTPUT);
    }
    attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
}

void loop() {
//  int pwm[4] = {2,2,2,2};
  int dir[4] = {0,1,1,1};
  if(millis() - prv_time[0] >=100){
    d_time = (millis()-prv_time[0])*0.001;
    enc2array(enc_count_0, enc_count_1, enc_count_2, enc_count_3);
//    vel_calculate(enc_value,4);
//    pid(15,6,8,3.0);
    pid(3.5,3,1,3.0);
    
    Serial.print("vel_t:");
    Serial.print(vel_t[0]);
    Serial.print(":");
    Serial.print("vel_flt:");
    Serial.print(vel_flt[0]);
    Serial.print(":");
    Serial.print("pwm:");
    Serial.print(pwm[0]);
    Serial.print(":");
    Serial.print("err:");
    Serial.print(err[0]);
    Serial.print(":");
    Serial.print("I_err:");
    Serial.print(I_err[0]);
    Serial.print(":");
    Serial.print("% err:");
    Serial.print(err_p[0]);
    Serial.print(":");
    Serial.print("D_err:");
    Serial.println(D_err[0]);
    prv_time[0] = millis();
    }  
   
   agv_run(pwm,dir,4);

}

void agv_run(int pwm_[],int dir_[],int n){
  for(int i = 0; i<1 ;i++){
    digitalWrite(DIR[i], dir_[i]);
    analogWrite(PWM[i], pwm_[i]);
  }
  
}

void vel_calculate(int enc_array[],int n){
    for(int i=0;i<2;i++){
      distance[i] = float(enc_array[i]-prv_enc_count[i])/TPM[0];
      vel_act[i] = (distance[i]/d_time);
    }
    for(int i=2;i<4;i++){
      distance[i] = float(enc_array[i]-prv_enc_count[i])/TPM[1];
      vel_act[i] = (distance[i]/d_time);
    }
    // Filter velocity at 25 Hz
    for(int i=0;i<4;i++){
      vel_flt[i] = 0.854*vel_flt[i]+0.0728*vel_act[i]+0.0728*vel_prv[i];
      vel_prv[i] = vel_flt[i];
      prv_enc_count[i] = float(enc_array[i]);
    }
 }

void pid(float kp,float ki,float kd,float err_max){
  vel_t[0] = 0.2;
  vel_t[1] = 0.2;
  vel_t[2] = 0.2;
  vel_t[3] = 0.2;
  
  vel_calculate(enc_value,4);
  for(int i=0;i<4;i++){

    err[i] = vel_t[i]-vel_flt[i];
    err_p[i] = err[i]*100/vel_t[i];
    I_err[i] = I_err[i] + err[i]*d_time; // 0.1*0.2
    D_err[i] = (err[i] - prv_err[i])/d_time;

    if(I_err[i] > vel_t[i]/0.08){
      I_err[i] = 0.2*I_err[i];
    }
    
    
    if(abs(err_p[i]) > err_max){
      pwm[i] = kp*err[i]+ki*I_err[i]+kd*D_err[i];
    }
    else
    {
      pwm[i] = pwm[i];
    }
    
    prv_err[i] = err[i];
    
    if(pwm[i] > 250){
      pwm[i]=250;
    }
    if(pwm[i] < -250){
      pwm[i]=-250;
    }
  }
}

void pid_clear()
{
  for(int i=0;i<4;i++){
    prv_err[i]=0;
    err[i]=0;
    I_err[i]=0;
    D_err[i]=0;
  }
}

void enc2array(int e0, int e1, int e2, int e3)
{
  enc_value[0] = e0;
  enc_value[1] = e1;
  enc_value[2] = e2;
  enc_value[3] = e3;
  
}

void enc_count0(){
  if (digitalRead(EB[0]) == 0)
  {
    if (digitalRead(EA[0]) == 0) 
    {
      enc_count_0++;
    }
    else
    {
      enc_count_0--;
    }
  }
}

void enc_count1(){
  if (digitalRead(EB[1]) == 0)
  {
    if (digitalRead(EA[1]) == 0) 
    {
      enc_count_1++;
    }
    else
    {
      enc_count_1--;
    }
  }
}

void enc_count2(){
  if (digitalRead(EB[2]) == 0)
  {
    if (digitalRead(EA[2]) == 0) 
    {
      enc_count_2++;
    }
    else
    {
      enc_count_2--;
    }
  }
}

void enc_count3(){
  if (digitalRead(EB[3]) == 0)
  {
    if (digitalRead(EA[3]) == 0) 
    {
     enc_count_3++;
    }
    else
    {
      enc_count_3--;
    }
  }
}
