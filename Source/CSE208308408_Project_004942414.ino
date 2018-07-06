#include <TimedAction.h>  // Protothreading
#include <Utility.h>      // Includes 'foreach' loop
#include <VirtualWire.h>  // Data transmission
#include <dht.h>          // Temperature, humidity sensor
dht DHT;

// Pin 12 - Dedicated TX pin
// Pin 11 - Dedicated RX pin
// Pin 10 - TX extra pin
#define led1 13   // LED's: LED1 controlled by light sensor,
#define led2 9    //  LED2 and LED3 controlled by light sensor
#define led3 8    //  and detection sensors
#define relay1 53   // Relays
#define relay2 52
#define relay3 51
#define relay4 50
#define relay5 49
#define lightsensor 7     // Light sensors
#define range_trig_NE 34  // Ultrasonic sensors, 2 for each package
#define range_echo_NE 35
#define range_trig_SE 32
#define range_echo_SE 33
#define DHT11_PIN 4     // Humidity sensor

// Keep id's 3-digit, set alternating tx_time manually
const int id = 188;
const int id_N = 187;
const int id_S = 189;
//if(id % 2 == 1) tx_time = 650;
//else tx_time = 850;
#define tx_time 850
const int tx_attempts = 10;   // TX attempts, set as higher value if reception not good
//const int coverage = 50;  // coverage of a motion sensor, in centimeters
const int coverage = 1;  // coverage of a motion sensor, in meters
const int distance_N = 10;  // Distance between this package and its northern neighbor
const int distance_S = 10;  // Distance between this package and its southern neighbor
const int humidity_avg = 20;    // Average humidity of the area
const int safety_time = 2000;   // Relay safety time, in ms
const int US_distance = 20;   // Distance from US sensor that causes high signal
const int noise_const = 200;// Tolerable amount of noise, in milliseconds

bool fog = 0;
int time_ms_N = 0;    // delay in milliseconds
int time_ms_S = 0;    // ""
int time_ms_rec = 0;  // ""
int time_start = 0;   // in milliseconds
bool delay_val = 0; // Validity of time delay information
int delay_rec = 5000;   // in milliseconds
int delay_start = 0;    // ""
int tx_it_N = 0;  // iterator
int tx_it_S = 0;   // iterator
int time_record_N = 0;    // in ms
int time_rec_N_start = 0; // ""
int time_record_S = 0;    // ""
int time_rec_S_start = 0; // ""
int relay_safety = safety_time;  // "", time before relay should change after last change
int relay_safety_start = 0; // ""
int noise_sensor = noise_const; // ""
int noise_sensor_start = 0;   // ""





void transmit() {   // Transmit function
  if(tx_it_N > 0) {     // If other functions have determined to send
    tx_it_N--;          //  information to the northern neighbor: display
    if(tx_it_N == 0) {  //  send message, build package, and send package.
      time_record_N = 0;
    }
    Serial.print("Sending to ");
    Serial.print(id_N);
    Serial.println("...");
    int val_N = 0;
    if(time_ms_N > 0) val_N = 1;
    char mes[8] = {'0','0','0','0','0','0','0','0'};
    mes[0] = (id_N / 256) % 16;         // Package: Contains the ID of the
    mes[1] = (id_N / 16) % 16;          //  receiver, the validity of the
    mes[2] = id_N % 16;                 //  value being sent (if value is
    mes[3] = val_N % 16;                //  0, value is not valid), and 
    mes[4] = (time_ms_N / 4096) % 16;   //  the value.
    mes[5] = (time_ms_N / 4256) % 16;
    mes[6] = (time_ms_N / 12) % 16;
    mes[7] = time_ms_N % 16;
    vw_send((uint8_t *)mes, /*strlen(mes)*/8);
  }

  if(tx_it_S > 0) {   // (Same function as for northern neighbor)
    tx_it_S--;
    if(tx_it_S == 0) {
      time_record_S = 0;
    }
    Serial.print("Sending to ");
    Serial.print(id_S);
    Serial.println("...");
    int val_S = 0;
    if(time_ms_S > 0) val_S = 1;
    char mes[8] = {'0','0','0','0','0','0','0','0'};
    mes[0] = (id_S / 256) % 16;
    mes[1] = (id_S / 16) % 16;
    mes[2] = id_S % 16;
    mes[3] = val_S % 16;
    mes[4] = (time_ms_S / 4096) % 16;
    mes[5] = (time_ms_S / 256) % 16;
    mes[6] = (time_ms_S / 16) % 16;
    mes[7] = time_ms_S % 16;
    vw_send((uint8_t *)mes, 8);
  }
}




void dht_check() {          // Checks temperature and humidity. If humidity is above
  bool val = 0;             //  a threshold, set the 'fog' boolean value as true.
  float hum = DHT.humidity; //  Then display temperature and humidity values.
  float tem = DHT.temperature;
  Serial.print("Humiture Sensor: ");
  switch (DHT.read11(DHT11_PIN)) {
    case DHTLIB_OK:
      Serial.println("OK\t");
      val = 1;
    break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("Checksum error\t");
    break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.println("Time out error\t");
    break;
    default:
      Serial.println("Unknown error\t");
    break;
  }
  if(val) {
    Serial.print("  Humidity: ");
    Serial.print(hum);
    Serial.println("%");
    if((hum - humidity_avg) > ((100 - hum) / 2)) fog = 1;
    else fog = 0;
    Serial.print("  Temperature(C): ");
    Serial.println(tem);
    tem = tem * 1.8;
    tem = tem + 32;
    Serial.print("  Temperature(F): ");
    Serial.println(tem);
    hum = 0;
    tem = 0;
  }
}

// Protothreading
TimedAction TX = TimedAction(tx_time, transmit);        // Transmits a message as often as 'tx_time specifies
TimedAction env_check = TimedAction(30000, dht_check);  // Checks for fog and other conditions every 30 seconds






void setup() {        // Set up Serial connection, transmission speed,
  Serial.begin(9600); //  receiver functionality, and Arduino pins
  vw_setup(4800);
  vw_rx_start();
  
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(relay5, OUTPUT);
  pinMode(lightsensor, INPUT);
  pinMode(range_trig_NE, OUTPUT);
  pinMode(range_echo_NE, INPUT);
  pinMode(range_trig_SE, OUTPUT);
  pinMode(range_echo_SE, INPUT);
}

void loop() {   // Every loop: try to receive information, update relevant
  RX();         //  times, perform its primarily functionality (described
  update_time();//  below), and check to see if it is time for other threads
  main_op();    //  (transmitter and condition check) to perform their functions
  TX.check();
  env_check.check();
}

void RX() {
  vw_wait_rx_max(random(100, 150));     // Maximum time the receiver will wait
  uint8_t buf[/*VW_MAX_MESSAGE_LEN*/8]; //  for a message.
  uint8_t buflen = /*VW_MAX_MESSAGE_LEN*/8;
  if (vw_get_message(buf, &buflen)) {     // If package is received, store its
    Serial.println("Packet received: ");  //  information.
    Serial.print("  ID Received = ");
    int id_rec = (((int)buf[0]) * 256) + (((int)buf[1]) * 16) + ((int)buf[2]);
    Serial.println(id_rec);
    Serial.print("  Validity: ");
    Serial.println(buf[3]);
    int time_rec = ((((int)buf[4]) * 4096) + (((int)buf[5]) * 256) + (((int)buf[6]) * 16) + ((int)buf[7]));
    Serial.print("  time_rec = ");
    Serial.println(time_rec);
    /*Serial.print("Original binary message: ");
    Serial.print(buf[0]);
    Serial.print(" ");
    Serial.print(buf[1]);
    Serial.print(" ");
    Serial.print(buf[2]);
    Serial.print(" / ");
    Serial.print(buf[3]);
    Serial.print(" / ");
    Serial.print(buf[4]);
    Serial.print(" ");
    Serial.print(buf[5]);
    Serial.print(" ");
    Serial.print(buf[6]);
    Serial.print(" ");
    Serial.println(buf[7]);*/
    if(id_rec == id) {    // If the package is meant for this mote,
      use_info(time_rec); //  use the info.
    }
  }
}

void main_op() {    // Primary operation
  if (digitalRead(lightsensor)) { // If there is no light, perform additional operation.
    digitalWrite(led1, HIGH);     //  If there is light, turn all lights off.
    relay_night_absence();   // Turn low lights on by default, check to see if person is nearby.
    if(tx_it_N == 0 && !presence_US(0)) {
      time_rec_N_start = millis();
    }
    if(tx_it_S == 0 && !presence_US(1)) {
      time_rec_S_start = millis();
    }
    if(!presence_US(1) && !presence_US(0) && (tx_it_S == 0) && (tx_it_N == 0) && (time_record_N > noise_const)) {
      tx_it_N = tx_attempts;
    }
    if(!presence_US(0) && !presence_US(1) && (tx_it_N == 0) && (tx_it_S == 0) && (time_record_S > noise_const)) {
      tx_it_S = tx_attempts;
    }
    if(fog) relay_fog();          // If there is fog, toggle fog setting.
    else if(delay_val) {          // If neighboring motes say a person should be near
      digitalWrite(led2, HIGH);   //  this mote, toggle its high lights
      digitalWrite(led3, HIGH);
      relay_night_presence();
    } else if (presence_US(1)) {    // If there is a person in front of the mote, turn
      digitalWrite(led2, HIGH);     //  on high lights. If there is a tolerable amount
      digitalWrite(led3, HIGH);     //  of traffic and noise, attempt to predict the 
      relay_night_presence();       //  path of the person and send information to the 
      time_record_S += (millis() - time_rec_S_start); // relevant mote.
      time_rec_S_start = millis();
    } else if(presence_US(0)) {
      digitalWrite(led2, HIGH);
      digitalWrite(led3, HIGH);
      relay_night_presence();
      time_record_N += (millis() - time_rec_N_start);
      time_rec_N_start = millis();
    } else {
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      relay_night_absence();
    }
    if(tx_it_N > 0 && !presence_US(0)) {
      float speed_calc = time_record_N / 1000;
      speed_calc = coverage / speed_calc;
      //speed_calc = speed_calc / 100;
      //Serial.print("**** Measured Time (N): ");
      //Serial.print(time_record_N);
      //Serial.println(" milliseconds");
      if(speed_calc < .1) {
        time_ms_N = 0;
        tx_it_N = 0;
      } else {
        speed_calc = 1 / speed_calc;
        speed_calc = speed_calc * distance_N;
        speed_calc = speed_calc * 1000;
        speed_calc = speed_calc - 2000; // Power lights 2 seconds before target arrives
        if(speed_calc > 0) time_ms_N = (int) speed_calc;
        else tx_it_N = 0;
      }
    }
    if(tx_it_S > 0 && !presence_US(1)) {
      float speed_calc = time_record_S / 1000;
      speed_calc = coverage / speed_calc;
      //speed_calc = speed_calc / 100;
      //Serial.print("**** Measured Time (S): ");
      //Serial.print(time_record_S);
      //Serial.println(" milliseconds");
      if(speed_calc < .1) {
        time_ms_S = 0;
        tx_it_S = 0;
      }
      else {
        speed_calc = 1 / speed_calc;
        speed_calc = speed_calc * distance_S;
        speed_calc = speed_calc * 1000;
        speed_calc = speed_calc - 2000; // Power lights 2 seconds before target arrives
        if(speed_calc > 0) time_ms_S = (int) speed_calc;
        else tx_it_S = 0;
      }
    }
  } else {
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    relay_day();
  }
}

bool presence_US(bool NE_SE) {  // 1 - NE, 0 - SE
  long duration;                  // Detects persense of a person or group
  int distance;                   //  of people in from of an ultrasonic sensor
  if(NE_SE) digitalWrite(range_trig_NE, LOW);
  else digitalWrite(range_trig_SE, LOW);
  delayMicroseconds(2);
  if(NE_SE) digitalWrite(range_trig_NE, HIGH);
  else digitalWrite(range_trig_SE, HIGH);
  delayMicroseconds(10);
  if(NE_SE) digitalWrite(range_trig_NE, LOW);
  else digitalWrite(range_trig_SE, LOW);
  if(NE_SE) duration = pulseIn(range_echo_NE, HIGH);
  else duration = pulseIn(range_echo_SE, HIGH);
  distance = duration / 58;
  if(distance <= US_distance) return 1;
  else return 0;
}

void use_info(int time_rec) { // Use time delay information received from neighboring motes
  if(time_ms_rec == 0 || (time_ms_rec != 0 && (time_rec) < time_ms_rec)) {
    time_ms_rec = time_rec;
    time_start = millis();
  }
}

void update_time() {    // Update relevant times
  if(delay_val) {
    delay_rec = delay_rec - (millis() - delay_start);
    delay_start = millis();
    if(delay_rec <= 0) {
      delay_val = 0;
      delay_rec = 5000;
    }
  }
  if(time_ms_rec != 0) {
    time_ms_rec = time_ms_rec - (millis() - time_start);
    time_start = millis();
    if(time_ms_rec <= 0) {
      delay_val = 1;
      delay_rec = 5000;
      delay_start = millis();
      time_ms_rec = 0;
    }
  }
  if(relay_safety != 0) {
    relay_safety = relay_safety - (millis() - relay_safety_start);
    relay_safety_start = millis();
    if(relay_safety <= 0) {
      relay_safety = 0;
    }
  }
  if(noise_sensor != 0) {
    noise_sensor = noise_sensor - (millis() - noise_sensor_start);
    noise_sensor_start = millis();
    if(noise_sensor <= 0) {
      noise_sensor = 0;
    }
  }
}

void relay_day() {  // Turns off all lights
  if(relay_safety == 0) {
    relay_safety = safety_time;
    relay_safety_start = millis();
    digitalWrite(relay5, HIGH);
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, HIGH);
    digitalWrite(relay4, HIGH);
  }
}

void relay_night_presence() { // Turns on high lights
  //if(relay_safety == 0) {
    relay_safety = safety_time;
    relay_safety_start = millis();
    digitalWrite(relay5, LOW);
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, HIGH);
    digitalWrite(relay4, LOW);
  //}
}

void relay_night_absence() {  // Turns on low lights
  if(relay_safety == 0) {
    relay_safety = safety_time;
    relay_safety_start = millis();
    digitalWrite(relay5, LOW);
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, HIGH);
    digitalWrite(relay4, HIGH);
  }
}

void relay_fog() {        // Turns on different set of high lights
  if(relay_safety == 0) { //  meant for fog.
    relay_safety = safety_time;
    relay_safety_start = millis();
    digitalWrite(relay5, LOW);
    digitalWrite(relay1, LOW);
    digitalWrite(relay4, HIGH);
    digitalWrite(relay3, LOW);
    digitalWrite(relay2, LOW);
  }
}



