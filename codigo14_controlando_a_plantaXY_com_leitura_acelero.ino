/*=================================
Definindo as portas do arduino Uno utilizadas para fazer a leitura de posição X,Y:
A0: Será usada como ADC e terá nela a posição Y do toque
A1: Será usada como ADC e terá nela a posição X do toque
D6 (porta 6): É o enable associado à chave ligada à porta A1. Permite atualizar o valor da posição X
D7 (porta 7): É o enable associado à chave ligada à porta A0. Permite atualizar o valor da posição Y 
D2 (porta 2): É uma das portas digitais usadas para fazer leitura de posição
D3 (porta 3): É uma das portas digitais usadas para fazer leitura de posição
D4 (porta 4): É uma das portas digitais usadas para fazer leitura de posição
D5 (porta 5): É uma das portas digitais usadas para fazer leitura de posição
=================================*/
// 

/*=================================
Definindo as portas do arduino Uno utilizadas para fazer ajustar o ângulo dos servo-motores:
D10 (porta 11): É o PWM usado para controlar o ângulo do servo motor que modifica o eixo X
D11 (porta 10): É o PWM usado para controlar o ângulo do servo motor que modifica o eixo Y
=================================*/
#include <Servo.h>
Servo myservoX;  // create servo object to control a servo
Servo myservoY;  // create servo object to control a servo
int angX = 60;   // Variável que será usada para ajustar o ângulo do myservoX para 93 + angX graus
int angY = 0;   // Variável que será usada para ajustar o ângulo do myservoY para 91 + angY graus


//Carrega a biblioteca Wire (para leitura do acelerometro)
#include<Wire.h>
//Endereco I2C do MPU6050
const int MPU=0x68;  
//Variaveis para armazenar valores dos sensores
long AcX,AcY;
int n=16; // talvez seja o número de médias
float EWMAx, EWMAy;
byte count = 0;


unsigned long tempo      = 0;
unsigned long tempo_proc = 0;    // tempo gasto com processamento de instruções
unsigned long Ts         = 17; // período de amostragem em ms  
unsigned long tempo_ref  = 0;    // Variável auxiliar de tempo para estabelecimento da referencia

int refX   = 0;
int refY   = 0;   


void setup()
{
   Serial.begin(2000000);
   pinMode(A1,INPUT);    // Fará leitura da posição X
   pinMode(A0,INPUT);    // Fará leitura da posição Y
   pinMode(6,OUTPUT);    // É o enable associado à chave ligada à porta A1. Permite atualizar o valor da posição X
   pinMode(7,OUTPUT);    // É o enable associado à chave ligada à porta A0. Permite atualizar o valor da posição Y 
   digitalWrite(6,LOW);  // abre a chave associada à porta A1
   digitalWrite(7,LOW);  // abre a chave associada à porta A0 

   myservoX.attach(11);        // attaches the servo on pin 11 to the servo object myservoX
   myservoY.attach(10);  // attaches the servo on pin 10 to the servo object myservoY 

   inicializacao_do_acelerometro();
}

void loop() 
{
  tempo = millis(); // tempo (em us) desde que o programa começou. Deve ser a primeira linha do loop
  // if (tempo-tempo_ref > 10000) { // se já faz mais que X ms que houve última mudança para ref=...
  //    refX = 150;
  //    refY = 150;
  // }
  // if (tempo-tempo_ref > 20000) { // se já faz mais que X ms que houve última mudança para ref=...
  //    refX = -150;
  //    refY = 150;
  // }
  // if (tempo-tempo_ref > 30000) { // se já faz mais que X ms que houve última mudança para ref=...
  //    refX = -150;
  //    refY = -150;
  // }
  // if (tempo-tempo_ref > 40000) { // se já faz mais que X ms que houve última mudança para ref=...
  //    refX = 150;
  //    refY = -150;
  //    tempo_ref = tempo;
  // }
  // refX = 100*sin(0.003*tempo); // T=20seg -> f=0.05Hz -> w=0.314rad/s -> w=0.000314rad/ms
  // refY = 100*sin(0.003*tempo+1.57); // idem anterior mas defasada em 1.57 rad = 90 graus

  leitura_angulos_acelerometro();
  // EWMAx = EWMAx *0.99+ (0.01 * float(AcX>>4));
  // AcX = 0;
  // EWMAy = EWMAy *0.99+ (0.01 * float(AcY>>4));
  // AcY = 0;
  Serial.print((-133.0+AcX/4.575), 1); Serial.print(" "); //angle in mº             # o -133.0 serve para calibrar o acelerômetro para que na posição perfeitamente horizontal a leitura do angulo X seja 0.
  Serial.print((-4.0+AcY/4.575), 1); Serial.print(" "); //angle in mº               # o -4.0 serve para calibrar o acelerômetro para que na posição perfeitamente horizontal a leitura do angulo Y seja 0.
  AcX=0; AcY=0;

  int X,Y; //Touch Coordinates are stored in X,Y variable
   
   // Fazendo a leitura da posição X
   pinMode(3,INPUT);     // D3 - high impedance
   pinMode(5,INPUT);     // D5 - high impedance
   pinMode(2,OUTPUT);    // D2
   digitalWrite(2,LOW);  // D2
   pinMode(4,OUTPUT);    // D4
   digitalWrite(4,HIGH); // D4
   delayMicroseconds(10); // Talvez esse delay seja necessário para que a nova configuração de portas fique de fato pronta
   digitalWrite(6,HIGH); // fecha a chave associada à leitura da posição X (para possibilitar a sua leitura)
   delay(5);            // Esse delay é necessário para que o capacitor tenha tempo de atualizar sua tensão
   X = analogRead(A1); //Reads X axis touch position 
   X = X-508;
   digitalWrite(6,LOW);  // abre a chave associada à leitura da posição X (para impedir a alteração do seu valor)

   Serial.print(refX); // Canal 1
   Serial.print(" ");

   Serial.print(X); // Canal 2
   Serial.print(" ");

   myservoX.writeMicroseconds(1480+angX);  
   
   Serial.print(angX); // Canal 3
   Serial.print(" ");

   // Fazendo a leitura da posição Y
   pinMode(2,INPUT);     // D2 - high impedance
   pinMode(4,INPUT);     // D4 - high impedance
   pinMode(3,OUTPUT);    // D3
   digitalWrite(3,LOW);  // D3
   pinMode(5,OUTPUT);    // D5
   digitalWrite(5,HIGH); // D5
   delayMicroseconds(10);
   digitalWrite(7,HIGH); // fecha a chave associada à leitura da posição Y (para possibilitar a sua leitura)
   delay(5);           // Esse delay é necessário para que o capacitor tenha tempo de atualizar sua tensão
   Y = analogRead(A0); //Reads Y axis touch position
   Y = Y-487;
   digitalWrite(7,LOW);  // abre a chave associada à leitura da posição Y (para impedir a alteração do seu valor)

   Serial.print(refY); // Canal 4
   Serial.print(" ");

   Serial.print(Y);
   Serial.print(" "); // Canal 5

   myservoY.writeMicroseconds(1550+angY);

   Serial.print(angY); // Canal 6


   tempo_proc = millis()-tempo; // tempo gasto com processamento de instruções

  if (tempo_proc<Ts){ // Caso o tempo de processamento esteja menor que o período de amostragem
    delay(Ts-tempo_proc); // aguarda X us. Implementa de forma aproximada 
                                      // o período de amostragem Ts.
    //Serial.print(millis()-tempo);
  }
  else { // Caso o perído de amostragem esteja maior que o desejado
    Serial.print("ALERTA: Ts maior que o especificado");
    Serial.print(" ");
    // A própria utilização do print abaixo aumenta o tempo de processamento e o período de amostragem efetivo.
    // Ele serve apenas para verificar o tempo de processamento das instruções.
    Serial.print(tempo_proc); // mostra o tempo aproximado gasto com o processamento de instruções 
    Serial.print(" ");

    // O print abaixo também influencia no período de amostragem efeito, pois o tempo que demora 
    // para o resultado da operação micros()-tempo ser transmitido via serial não está sendo
    // levado em conta.
    Serial.print(micros()-tempo); // mostra o período de amostragem efetivo aproximado
  }
  
  Serial.println(" "); // Deve ser a última linha do loop
}













void inicializacao_do_acelerometro()
{
 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);     //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x6C);     //Inicializa o MPU-6050
  Wire.write(0x0F); 
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1A); 
  Wire.write(6); //digital low pass filter = 5 Hz
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);     //define accel scale 2g
  Wire.write(0); 
  Wire.endTransmission(true);
}

void leitura_angulos_acelerometro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,8,true); //read 8 bytes from MPU-6050
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX +=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY +=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
}