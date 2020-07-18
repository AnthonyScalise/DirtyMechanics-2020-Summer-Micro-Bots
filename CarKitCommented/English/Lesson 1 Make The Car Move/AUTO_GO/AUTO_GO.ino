/*
    This program allows you to create an atonomus path with pre built commands for movement

    Some information to note:
        * A line that starts with the characters:  //  :is a comment. Commnets are ignored when the program is complied.

        * A block(a group of code) that that is contained within:  ______________________  :is a function.
           functions can be used to acess a group of code         |                      |
           from a single command                                  | void "someName"() {  |
                                                                  |                      |
                                                                  |                      |
                                                                  | }                    |
                                                                  |                      |
                                                                   ----------------------
*/


//The direction of the car's movement table for refference
//    ENA   ENB   IN1   IN2   IN3   IN4   Description
//    HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is runing forward
//    HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is runing back
//    HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
//    HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
//    HIGH  HIGH  LOW   LOW   LOW   LOW   Car is stoped
//    HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is stoped
//    LOW   LOW   N/A   N/A   N/A   N/A   Car is stoped


/*
    A (#define "name" "value")  will bind the value to the name so that
    anywhere the name is used it will be replaced with the value when compiled.
    This is usefull for a value like a pin number which is used multiple times
    in the code. If the number needs to be changed you can change all instances by
    just chaning its definition.
*/
//define L298n module IO Pin
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// This is the function to make the robot move forwards
void forward(){
  //Each:  digitalWrite("pin number definition", "state")  :sets the pin number(or definition) given to the state(HIGH / LOW) given
  digitalWrite(ENA,HIGH);     //enable L298n A channel
  digitalWrite(ENB,HIGH);     //enable L298n B channel
  digitalWrite(IN1,HIGH);     //set IN1 hight level
  digitalWrite(IN2,LOW);      //set IN2 low level
  digitalWrite(IN3,LOW);      //set IN3 low level
  digitalWrite(IN4,HIGH);     //set IN4 hight level
  Serial.println("Forward");  //send message to serial monitor
}

// This is the function to make the robot move backwards
void back(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Back");
}

// This is the function to make the robot turn left
void left(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  Serial.println("Left");
}

// This is the function to make the robot turn right
void right(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Right");
}

//before execute loop() function,
//setup() function executes first and will only execute once.
//This is where you would do any initialization for your robot
void setup() {
  Serial.begin(9600);   //open serial and set the baudrate. This allows you to use the serial monitor for debuging.
  pinMode(IN1,OUTPUT);  //pinMode allows you to set a pin on the arduino to either an INPUT or an OUTPUT
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
}

//The loop funtion contains the main code that is run repetitively.
void loop() {
  forward();    //go forward
  delay(1000);  //delay 1000 milliseconds
  back();       //go back
  delay(1000);  //delay 1000 milliseconds
  left();       //turning left
  delay(1000);  //delay 1000 milliseconds
  right();      //turning right
  delay(1000);  //delay 1000 milliseconds
}
