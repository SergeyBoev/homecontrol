
struct 
{
  byte ModBusID = 1;
  
} InitParams;

class cLamp {
  private:
    byte pinLamp;
    byte pinSW;
    enum state {ON,OFF};
    enum state swState;
    enum state GetSwState(){
      if (digitalRead(pinSW)==LOW) return ON;
      else return OFF;
    }
  public:
    cLamp(int L, int S) { 
      pinLamp=L; pinMode(L,OUTPUT); 
      pinSW=S; pinMode(S,INPUT); digitalWrite(S, HIGH);
    };
    void LampON(){digitalWrite(pinLamp, HIGH);};
    void LampOFF(){ digitalWrite(pinLamp, LOW);};
    void run(){
      if (GetSwState() != swState){
        swState=GetSwState();
        if (swState == ON) LampON();
        else LampOFF();
      };
    };
};


//cDoor Door(SensorPin, LockPin);
//cDoor Window(SensorPin, LockPin);

cLamp L1(3,4);//Лампа 1
cLamp L2(5,6);//Лампа 2
//cSensor Temp;               //Датчик температуры
//cSensor Humid;              //Датчик влажности

void setup() {
  

}

void loop() {
  L1.run();
  L2.run();
}
