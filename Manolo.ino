/**----------Manolo self balancing robot v.1-----------------------
 * codigo echo por Elchals el verano del 2015
 * usa la App gratuita Ardudroid para modificar las variables del PID por bluetooth
 * tambien usa la App gratuita Joystick BT commander para controlar los movimientos del robot por bluetooth
 * provado con un arduino uno , motores paso a paso , driveres pololu , bluetooth hc-05 y IMU mpu6050
 * no soy programador conque seguramente el codigo sea muy mejorable , pero a mi me ha funcionado 
 * https://www.youtube.com/watch?v=tQ3afodyaBc
 */







#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"

//CONFIGURACION PINES
#define LED 12

//COMPROVADOR VOLTAGE
#define VOLTAJE A0
#define VOL_MIN 10
#define VOL_CHECK 1    //1:si 0:no

//TIMERS
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
float tiempoActual,tiempo;
int contador;

//MOTORES
#define ZERO_SPEED 255
#define MAX_ACCEL  70
#define VEL_MAX    500
int  dir_M1=1,dir_M2=1;  //Direccion motores  
float timer_period1,timer_period2;
int16_t speed_M1,speed_M2; 
float vel1 , vel2;  
float velocidadMedia, velocidad;
float giro=0;
int velocidadDeseada = 0;
int V = 10;


//BLUETOOTH ARDUDROID Y JOYSTICK
SoftwareSerial BT1(10, 11); // RX | TX
#define    STX          0x02
#define    ETX          0x03
#define START_CMD_CHAR '*'
#define END_CMD_CHAR '#'
#define DIV_CMD_CHAR '|'
#define CMD_DIGITALWRITE 10
#define CMD_ANALOGWRITE 11
#define CMD_TEXT 12
#define CMD_READ_ARDUDROID 13
#define MAX_COMMAND 20  // max command number code. used for error checking.
#define MIN_COMMAND 10  // minimum command number code. used for error checking. 
#define IN_STRING_LENGHT 40
#define MAX_ANALOGWRITE 255
#define PIN_HIGH 3
#define PIN_LOW 2
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};  
int joyX,joyY;
int comando;
float joyXX,joyYY;

//MPU 6050 Acelerometro+Giroscopio
#define MPU 0x68
#define MAX_ANG 15
MPU6050 accelgyro; 
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
double gyroScale = 131;
int16_t ax, ay, az, gx, gy, gz;
int muestras = 3; 
float anguloActual, anguloObjetivo;
float angulo,angObj;
float H = 0.5;

//CONTROL PID
float Kp, Ki , Kd , Kps ,Kis ,Kds;
float integralSum, PID_errorSum;
float errorAnterior, errorAnt;
#define ITERM_MAX_ERROR 25   
#define ITERM_MAX 8000  



void setup()
{
  
  pinMode(4,OUTPUT);  // STEP MOTOR 1 PORTD,4          
  pinMode(5,OUTPUT);  // DIR MOTOR 1  PORTD,5
  pinMode(7,OUTPUT);  // STEP MOTOR 2 PORTD,7
  pinMode(6,OUTPUT);  // DIR MOTOR 2  PORTD,6              
  pinMode(8,OUTPUT);  // ENABLE PORTB,0
  digitalWrite(8,HIGH);
  
  Serial.begin(115200);
  Wire.begin();
  BT1.begin(57600);

  while(BT1.available())  BT1.read();         //vaciar RX buffer

 
  

  inicializar_variables();
  
  pinMode(LED,OUTPUT);
  pinMode(VOLTAJE,INPUT);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

 
  accelgyro.setXAccelOffset(-1249);
  accelgyro.setYAccelOffset(-818);
  accelgyro.setZAccelOffset(1478);
  accelgyro.setXGyroOffset(63);
  accelgyro.setYGyroOffset(-2);
  accelgyro.setZGyroOffset(-31);

  
  
  TCCR1A = 0;                                              // Timer1 CTC mode 4
  TCCR1B = (1<<WGM12)|(1<<CS12);                           // Prescaler=256
  OCR1A = 255;                                             // Motor parado
  TCNT1 = 0;              
                                
  TCCR2A = (1<<WGM21);                                     // Timer2 CTC mode 4
  TCCR2B = (1<<CS22)|(1<<CS21);                            // Prescaler=256
  OCR2A = 255;                                             // Motor parado
  TCNT2 = 0;  
              
  dir_M1 = 0;   
  dir_M2 = 0;
  
  for(int i=0;i<100;i++)
    {
        float tiempoAnterior = tiempoActual;
        tiempoActual = micros();
        float tiempoPasado = (tiempoActual - tiempoAnterior);                 
        tiempo  = tiempoPasado / 1000;                                       
        anguloActual = calcAng(tiempo);                                            //Angulo inicial.
    }
    
   
  digitalWrite(LED,HIGH);
 
  Serial.println("OK!!!");

  tiempoActual = micros();

  TIMSK1 |= (1<<OCIE1A);
  TIMSK2 |= (1<<OCIE2A);
}

void loop()
{
   #if VOL_CHECK==1
    compVoltaje();
   #endif

   float tiempoAnterior = tiempoActual;
   tiempoActual = micros();
   float tiempoPasado = (tiempoActual - tiempoAnterior);                 //tiempo pasado desde ciclo anterior en microsegundos
   tiempo  = tiempoPasado / 1000;                                        //tiempo del ciclo en milisegundos
   anguloActual = calcAng(tiempo);
   
   float velo = ((vel1 + vel2) / 2);
   velocidadMedia = velo;//0.95 * velocidadMedia + 0.05 * velo;
   angObj = -speedPIControl(tiempo, velocidadMedia, velocidadDeseada, Kps, Kis ,Kds) ;
   anguloObjetivo = 0.02 * angObj + 0.98 * anguloObjetivo;
   anguloObjetivo = constrain (anguloObjetivo ,-MAX_ANG,MAX_ANG);
   float resultadoPID = PID (anguloActual, anguloObjetivo, Kp, Kd, Ki, tiempo);
   
  //Serial.print("   Tiempo:");Serial.println(tiempo);
   
   if ((anguloActual<55)&&(anguloActual>-55))  // Esta el robot de pie????
    { 
      if (anguloActual<(anguloObjetivo+H) && anguloActual>(anguloObjetivo-H) && (velocidad<V && velocidad>-V)) 
      {
        CLR(PORTB,0);
        vel1 = 0 + giro ;
        vel2 = 0 - giro ;
        integralSum = 0;
        setMotorSpeedM1(vel1);
        setMotorSpeedM2(vel2);
      }  else
      {
         CLR(PORTB,0);
         velocidad = resultadoPID;
         vel1 = velocidad + giro ;
         vel2 = velocidad - giro ;
         vel1 = constrain(vel1,-VEL_MAX,VEL_MAX);
         vel2 = constrain(vel2,-VEL_MAX,VEL_MAX);
         setMotorSpeedM1(vel1);
         setMotorSpeedM2(vel2);
     /** Serial.print("  Angulo:");
      Serial.print(anguloActual);
      Serial.print("  PID:");
      Serial.print(velocidad);
      Serial.print("  Vel1:");
      Serial.print(timer_period1);
      Serial.print("  Vel2:");
      Serial.println(timer_period2);
      **/
    }}
      else
      {
        SET(PORTB,0);
        setMotorSpeedM1(0);
        setMotorSpeedM2(0);
        integralSum = 0;
        PID_errorSum = 0;
      }
   
       
       bluetooth();
     
   //Serial.print("VELOCIDAD:");Serial.print(velocidad);Serial.print("    VEL1:");Serial.print(vel1);Serial.print("    VEL2:");Serial.print(vel2);Serial.print("    T1:");Serial.print(timer_period1);Serial.print("    T2:");Serial.println(timer_period2);





}


void compVoltaje(){
  float voltaje = analogRead(VOLTAJE);
  voltaje = voltaje * (5.0 / 1023.0)*3;
  if (voltaje<VOL_MIN) 
  {
      while(1)
      {
        delay(1000);
        digitalWrite(LED,HIGH);
        delay(100);
        digitalWrite(LED,LOW);
        delay(100);
        digitalWrite(LED,HIGH);
        delay(100);
        digitalWrite(LED,LOW);
      }
  }   
}



ISR(TIMER1_COMPA_vect)
{
 if (dir_M1==0) 
    return;
  SET(PORTD,4);  //Genero un paso del motor1
  delay_1us();
  delay_1us();
  CLR(PORTD,4); 
  
}

ISR(TIMER2_COMPA_vect)
{
  if (dir_M2==0)  
    return;
  SET(PORTD,7);  //Genero un paso del motor2
  delay_1us();
  delay_1us();
  CLR(PORTD,7);
  
}



float calcAng (float tp) {
 
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                          //lecturas del acelerometro y giroscopio
   gsx = gx/gyroScale;                                                          //escalando la lectora del giroscopio en eje X 
   ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));             //calculo de angulo eje Y con el acelerometro
   angulo = (0.01 * ary) + (0.99*(angulo + (gsx*tp/1000)));               //juntando valores del giroscopio y acelerometro con un filtro complemntario
   //Serial.println(ary);
   return(angulo);
}




void delay_1us()  
{
  __asm__ __volatile__ (
  "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}



int calcVel (float x)              //0-500
    {   
        //float maxi = 20833.333333333333;
        float mini = 210;
        float vel = 20623.333333333333;
        vel = vel / 500;
        vel = (vel * x) + mini;
        vel = 1 / vel;
        vel = vel / 0.000016;
        return ((int)vel-1);                
        }

void setMotorSpeedM1(int tspeed)
{ 
 
  int16_t speed1;

  if ((speed_M1 - tspeed)>MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed)<-MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;


  speed1 = speed_M1;  

  if (speed1==0) {
    timer_period1 = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed1>0)
  {
    timer_period1 = calcVel(speed1);   
    dir_M1 = 1;
    SET(PORTD,5);  
   
  }
  else
  {
    timer_period1 = calcVel(-speed1);
    dir_M1 = -1;
    CLR(PORTD,5);  // Dir Motor 1
    
    }
    
  if (timer_period1>255) {timer_period1 = ZERO_SPEED;}
   
  OCR1A = timer_period1;  
  
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
}

void setMotorSpeedM2(int tspeed)
{ 
 
  int16_t speed2;

  if ((speed_M2 - tspeed)>MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed)<-MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;


  speed2 = speed_M2;  

  if (speed2==0)
  {
    timer_period2 = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed2>0)
  {
    timer_period2 = calcVel(speed2);  
    dir_M2 = 1;
    CLR(PORTD,6);    
  }
  else
  {
    timer_period2 = calcVel(-speed2);
    dir_M2 = -1;
    SET(PORTD,6);   
  }
  if (timer_period2 > 255)   
    timer_period2 = ZERO_SPEED;

  OCR2A = timer_period2;  
  
  if (TCNT2 > OCR2A)
    TCNT2 = 0;
}



float PID (float anguloActual, float anguloDeseado, float Kp, float Kd, float Ki, float tiempo) {
  float errorActual;
  float salida;
  float output;
  
  errorActual = anguloDeseado - anguloActual;
  float proporcional = errorActual * Kp * 0.1;
  integralSum = integralSum + errorActual ;
  float integral = Ki * integralSum * tiempo * 0.001;
  integral = constrain (integral, -VEL_MAX, VEL_MAX);
  float derivativo = Kd * gsx / tiempo ;
  errorAnterior = errorActual;
  output = constrain((proporcional - derivativo + integral), -VEL_MAX, VEL_MAX);
  //if (tiempo > 10000) {Serial.print("KP:");Serial.print(proporcional);Serial.print("    Ki:");Serial.print(integral);Serial.print("   KD:");Serial.print( derivativo);Serial.print("   OUT:");Serial.println(output);
 // }
  return (output);
}



float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki, float Kd)
{
  float error;
  float output;

  error = setPoint-input;
  PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);
  
  //Serial.println(PID_errorSum);
  float prop = Kp * error;
  float integ = Ki * PID_errorSum * DT * 0.001;
  float deriv = Kd * (error - errorAnt) / DT;
  output = prop + integ + deriv;
  errorAnt = error;
  //Serial.print("   KPS:");Serial.print(prop);Serial.print("   KIS:");Serial.print(integ);Serial.print("  KD:");Serial.print(deriv);Serial.print("  OUT:");Serial.print(output);Serial.print("    ANGULO:");Serial.println(anguloObjetivo);
  return(output);
}


void bluetooth() {

if(BT1.available())  {                           // data received from smartphone
   
   cmd[0] =  BT1.read(); //Serial.write( cmd[0]);;return;
   if(cmd[0] == STX)  {
         int i=1;      
         while(BT1.available())  {
              
              cmd[i] = BT1.read();
              if(cmd[i]>127 || i>7)                 break;     // Communication error
              if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
              i++;
              }
         if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
         else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
   }
 
        int ard_command = 0;
        int pin_num = 0;
        int pin_value = 0;
  
         if (cmd[0] != 42)   
             return; // Si no hay comando, vuelta a empezar
    
        ard_command = BT1.parseInt(); // Leemos la orden
        delay(25);
        pin_num = BT1.parseInt();     // Leemos el pin
        delay(25);
        pin_value = BT1.parseInt();   // Leemos valor

        if (ard_command == CMD_TEXT)     // Si el comando es de texto:
          {  
              String s = GetLine();
          if (s == "h") {
             BT1.print("kp,ki,kd,kps,kis");
             comando=0;
             return;
          }
                  
          if (s=="s") {
          comando=0;
          guardar_datos ();
          return;
        }
                        
        if (s=="kp") {
          comando=1;
          BT1.print("     Kp:");BT1.print(Kp);
          return;
        } 
        
        if (s=="ki") {
          comando=2;
          BT1.print("     Ki:");BT1.print(Ki);
          return;
        } 
        
        if (s=="kd") {
          comando=3;
          BT1.print("     Kd:");BT1.print(Kd);
          return;
        } 
                
        if (s=="kps") {
          comando=4;
          BT1.print("     Kps:");BT1.print(Kps);
          return;
        } 
        
        if (s=="kis") {
          comando=5;
          BT1.print("     Kis:");BT1.print(Kis);
          return;
        } 

        if (s=="kds") {
          comando=6;
          BT1.print("     Kds:");BT1.print(Kds);
          return;
        } 
        
        if (comando==1) {
          Kp = string_float(s);
          BT1.print("     Kp:");BT1.print(Kp);
          return;
        }
        
        if (comando==2) {
          Ki = string_float(s);
          BT1.print("     Ki:");BT1.print(Ki);
          return;
        }
        
        if (comando==3) {
          Kd = string_float(s);
          BT1.print("     Kd:");BT1.print(Kd);
          return;
        }
        
        if (comando==4) {
          Kps = string_float(s);
          BT1.print("     Kps:");BT1.print(Kps);
          return;
        }
        
        if (comando==5) {
          Kis = string_float(s);
          BT1.print("     Kis:");BT1.print(Kis);
          return;
        }
        if (comando==6) {
          Kds = string_float(s);
          BT1.print("     Kds:");BT1.print(Kds);
          return;
        }
                
          } 
         /**if (ard_command == CMD_READ_ARDUDROID) 
             {   BT1.print(" Analog 0 = "); 
                 BT1.println(analogRead(A0));  // Leemos A0
                 return;  // Done. return to loop();   
             }
         if (ard_command == CMD_DIGITALWRITE)
             { processDW(pin_num, pin_value);
               return;
             }
             
        if (ard_command == CMD_ANALOGWRITE) 
           {  analogWrite(  pin_num, pin_value ); 
             // add your code here
             return;  // De vuelta alloop();
           }**/
    }
}

void processDW(int pin_num, int pin_value)
{   if (pin_value == PIN_LOW) 
        pin_value = LOW;
    else if (pin_value == PIN_HIGH)
        pin_value = HIGH;
    else 
        return; // error in pin value. return. 
        
    digitalWrite( pin_num,  pin_value);   
    return;  
}

String GetLine()
   {   String S = "" ;
       if (BT1.available())
          {    char c = BT1.read(); 
                delay(25) ;
                c = BT1.read();
                while ( c != END_CMD_CHAR)            //Hasta que el caracter sea END_CMD_CHAR
                  {     S = S + c ;
                        delay(25) ;
                        c = BT1.read();
                  }
                return( S ) ;
          }
   }
  


 
 float string_float(String cadena) //Metodo que regresa un numero recibiendo una cadena
{
  char nume[50];  //vector temporal para guardar numero
  cadena.toCharArray(nume,50);  //se convierte la cadena en char y se guarda en nume
  float numero=atof(nume);  //convertimos el vector char en numero
  return numero;  //returnamos el numero
}





void getJoystickState(byte data[8])    {
  
 joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
 joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
 joyX = joyX - 200;                                                  // Offset to avoid
 joyY = joyY - 200; // transmitting negative numbers
 if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
 joyXX = joyX * joyX /100;
 joyYY = joyY  * 2;
 
 giro = joyXX;
 velocidadDeseada = joyYY;


  /** Serial.print("Joystick position:  ");
   Serial.print(joyXX);  
   Serial.print(", ");  
   Serial.println(joyYY);**/
}

void getButtonState(int bStatus)  {
 switch (bStatus) {
// -----------------  BUTTON #1  -----------------------
   case 'A':
        SET(PORTB,0);
        setMotorSpeedM1(0);
        setMotorSpeedM2(0);
        integralSum = 0;
        PID_errorSum = 0;
        while(1);
     break;
   case 'B':
     
     break;

// -----------------  BUTTON #2  -----------------------
   case 'C':
     
     break;
   case 'D':
     
     break;

/** -----------------  BUTTON #3  -----------------------
   case 'E':
     
   case 'F':
     
     break;

// -----------------  BUTTON #4  -----------------------
   case 'G':
     
     break;
   case 'H':
     
    break;

// -----------------  BUTTON #5  -----------------------
   case 'I':           // configured as momentary button
     
     break;
   case 'J':
 
     break;

// -----------------  BUTTON #6  -----------------------
   case 'K':
    
    break;
   case 'L':
     
     break;
 }
// ---------------------------------------------------------------
**/
}}


void escrivir_eeprom(int direccion, float num)
      {
      long valor=num*10000;
      
      byte cuatro = (valor & 0xFF);
      byte tres = ((valor >> 8) & 0xFF);
      byte dos = ((valor >> 16) & 0xFF);
      byte uno = ((valor >> 24) & 0xFF);

      EEPROM.write(direccion, cuatro);
      EEPROM.write(direccion + 1, tres);
      EEPROM.write(direccion + 2, dos);
      EEPROM.write(direccion + 3, uno);
      }

float leer_eeprom(long direccion)
      {
      
      long cuatro = EEPROM.read(direccion);
      long tres = EEPROM.read(direccion + 1);
      long dos = EEPROM.read(direccion + 2);
      long uno = EEPROM.read(direccion + 3);

      float num=((cuatro << 0) & 0xFF) + ((tres << 8) & 0xFFFF) + ((dos << 16) & 0xFFFFFF) + ((uno << 24) & 0xFFFFFFFF);
      return (num/10000);
      }

void guardar_datos () 
      {
        
      
        escrivir_eeprom (0,Kds);
        escrivir_eeprom (4,Kp);
        escrivir_eeprom (8,Ki);
        escrivir_eeprom (12,Kd);
        escrivir_eeprom (16,Kps);
        escrivir_eeprom (20,Kis);
       
        
        BT1.println("Datos guardados en EEPROM OK");
      }
      
void inicializar_variables()
      {
        
        Kds = leer_eeprom(0);
        Kp = leer_eeprom(4);
        Ki = leer_eeprom(8);
        Kd = leer_eeprom(12);
        Kps = leer_eeprom(16);
        Kis = leer_eeprom(20);
         
        BT1.println("Variables inicializadas");
      }
