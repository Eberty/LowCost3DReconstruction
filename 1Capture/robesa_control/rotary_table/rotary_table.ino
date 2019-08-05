//Programa : Controle da Robesa

//Carrega a biblioteca do encoder
#include <RotaryEncoder.h>

//Define constantes
#define DEPTH 41
#define SR 42
#define COLOR 43
#define ALL 44
#define UP 45
#define DOWN 46
#define STOP 200

//Pinos de ligacao do encoder
RotaryEncoder encoder(A2, A3);

//Variaveis para encoder
int valor = 0;
int newPos = 0;
float constEnc = 0.44444;


//Pinagem
int senH = 5;
int senAH = 7;
int pwm = 6;
int botEnc = 4;

//Variáveis
float passoAng;
int comando;
int passoInt;
float actAng;
int numCap;
int i = 1;
int newAng;
int actPos = 0;
bool parado = false;
int timeStop;
int timeRef;
int timeLimit;
int atraso;
int lixo;

//--------------------//--------------------//--------------------//--------------------//--------------------//--------------------//--------------------//--------------------//

void setup()
{
  
  //Configuraçao dos pinos
  pinMode(senH,OUTPUT);
  pinMode(senAH,OUTPUT);
  pinMode(pwm,OUTPUT);
  pinMode(botEnc, INPUT);
  
  //Inicializa pinos
  digitalWrite(pwm,HIGH);
  digitalWrite(senAH,LOW);
  digitalWrite(senH,LOW);
  digitalWrite(pwm,HIGH);
    
  Serial.begin(9600);
  
  //Escolha de parametros de scaneamento
  Serial.println("Deseja girar 360 graus completos ou por passo a passo? (Digite 1)");
  while (Serial.available() == 0){}
  comando = Serial.parseInt();  
  if (comando == 1){
    Serial.println("Quantos malhas pretende capturar?");
    while(Serial.available() == 0){}
    numCap = Serial.parseInt();
    if (360%numCap!=0){
      passoInt = (360/numCap + 1)/constEnc;
    }
    else passoInt =(int)360/(numCap*constEnc);
    passoAng = passoInt*constEnc;
    Serial.print("Cada passo tera exatamente ");
    Serial.print(passoAng);
    Serial.println(" graus");
    delay(2000);
  }
 /* else if (comando == 2){
    Serial.println("Qual o ângulo de avanço?");
    Serial.read(numCap);
    passoInt = numCap/constEnc;
  }   */
  newAng = passoInt;
}

//--------------------//--------------------//--------------------//--------------------//--------------------//--------------------//--------------------//--------------------//

void loop(){
  
  while(actPos<360/constEnc && i<=numCap){
    
  //Verifica se o botao do encoder foi pressionado
  valor = digitalRead(botEnc);
  if (valor != 1)
  {
    Serial.println("Alerta de falha -> Botao pressionado");
    while (digitalRead(botEnc) == 0)
      delay(10);
  }
    
  //Enquanto nao se chega na posicao, motor segue ligado
  if (actPos < newAng)
       digitalWrite(senH,HIGH);
       
       //Se chegou na posiçao, scaneia e continua
       else{
         
         //Freio
         digitalWrite(senAH,HIGH);
         delay(200);
         
         //Desliga
         digitalWrite(senAH,LOW);
         digitalWrite(senH,LOW);
         
         //Interaçao com kinect
         if (i<numCap)
          Serial.println("Digite algo para continuar");
         while (Serial.available() == 0){}
         lixo = Serial.parseInt();
         
         //Liga e define novo angulo de parada
         digitalWrite(senH,HIGH);
         i++;
         newAng = i*passoInt;
         timeRef = millis();
         parado = false;
       };
       
  //Le as informacoes do encoder
  static int pos = 0;
  encoder.tick();
  
  int newPos = encoder.getPosition();
  
  //Se a posicao foi alterada, mostra o valor no Serial Monitor
  if (pos != newPos) {
    actAng = (float) newPos*constEnc;
    Serial.print(actAng);
    Serial.println();
    pos = newPos;
    parado = false;
  //Se a posiçao permanece a mesma, verifica falha do motor
  } else if (pos == newPos && parado){
      timeStop = millis();
      timeLimit = timeStop - timeRef;
      if (timeLimit > STOP){
        Serial.println("ALERTA DE TRAVAMENTO!!!");
        //Freio
        digitalWrite(senAH,HIGH);
        delay(200);
        
        //Desliga
        digitalWrite(senAH,LOW);
        digitalWrite(senH,LOW);
        Serial.println("Colocar um tempo de espera");
        while (Serial.available() == 0){}
        atraso = Serial.parseInt();
        Serial.print("Funcionamento normal em ");    
        Serial.print(atraso);
        Serial.print(" segundos");
        atraso *= 1000;
        delay(atraso);
        timeRef = millis();
        parado = false;
      }
  } else if (pos == newPos && !parado){
      parado = true;
      timeRef = millis();
  }
  actPos = newPos;
}
  digitalWrite(senAH,LOW);
  digitalWrite(senH,LOW);
  if (i == numCap)
      Serial.print("Escaneamento Completo!");  
  while(Serial.available() == 0){} 
}




