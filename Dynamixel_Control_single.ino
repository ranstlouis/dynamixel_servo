/*
Dynamixel_Control.ino
  written by ranstlouis
  11/3/21
  *****************************************************************************
  Decription:
  This library enables simple control of a single dynamixel servo via commands.
  Dependencies:
   - SoftHalfDuplexSerial library (https://github.com/akira215/HalfDuplexSerial-for-Arduino)
   - DynamixelAx library (https://github.com/akira215/DynamixelArduino)
  Hardware:
   - This example uses the SoftHalfDuplexSerial library, so check that the connected data pin supports change interrupts. Pin 2 is used here
   - Check that the board is properly grounded with dynamixel power supply.
   - The servo's baudrate must be changed from its default. Use the "DynamixelBaudSwitch.ino" sketch to switch it from 1000000bps to 57600bps.
*/
#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>


int servo_comm_pin = 2;
int servo_id = 1;


softHalfDuplexSerial port(servo_comm_pin); // data pin 
dxlAx dxlCom(&port);

String _readString;         // Input string from serial monitor
bool _strComplete = false;
int _id = 1;                // Default Dynamixel servo ID

unsigned int setpoint;
unsigned int curr_pos;
unsigned int max_pos;
unsigned int min_pos;
unsigned int speed;
unsigned int torque;
bool         cont_hold;

bool new_setpoint = false;
unsigned int units = 1;
bool isMoving; //bool to keep track of movement

bool echo_global = true;
unsigned long curr_time = 0;
bool data_on = false;
unsigned short error = DXL_ERR_SUCCESS; //Initialize error variable

void printServoId(String msg);
void printDxlResult();
void printDxlError(unsigned short dxlError);

//Initialize functions for getting results
void clear_result();
int  get_result();


unsigned int pos_init = 400;
unsigned int max_pos_init = 512; // 90 degrees
unsigned int min_pos_init = 203; // ~0 degrees
float pi = 3.14159265;

void setup() {
  // Open serial communications and wait for port to open (PC communication)
  Serial.setTimeout(30);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("_StartingCOM!");
  // Start the communication with the dynamixels
  dxlCom.begin(57600);
  
  // Initialize variable vectors
  
  setpoint = pos_init;
  dxlCom.readPresentPosition(servo_id);
  curr_pos = get_result(); // Gives the result of the most recent "read". In this case, the present position
  max_pos  = max_pos_init;
  min_pos  = min_pos_init;
  speed    = 1023;         //Start with max speed
  torque   = 1023;         //Start with max torque
  cont_hold   = false;     //Start with no continuous hold

  dxlCom.setTorqueLimit(servo_id,torque);
  clear_result(); //Prep dynamixel for new command
  dxlCom.setMovingSpeed(servo_id,speed);
  clear_result();
  dxlCom.setGoalPosition(servo_id,setpoint);
  clear_result(); 
  
  isMoving = true; 
}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  // Run the move if there's a new setpoint
  if (new_setpoint){
    new_setpoint=false;
    isMoving = true; //Motors are moving now
    // Set the new goal positions    
    dxlCom.setGoalPosition(servo_id,setpoint); //Set new goal     
    get_result(); //reset and get result   

    delay(10);

    // Wait until the motor finishes moving    
    while (isMoving){ 
      //Check current position
      while(dxlCom.isBusy()); // waiting the status return delay time
      dxlCom.readPresentPosition(servo_id); //Read the current motor position
      while(!dxlCom.dxlDataReady());        // waiting the answer of servo          
      curr_pos = get_result(); //Returns the result of previous non-error command          
      
      //Test if moving
      while(dxlCom.isBusy()); 
      dxlCom.isMoving(servo_id);
      while(!dxlCom.dxlDataReady());              
      isMoving = get_result();    
            
      curr_time = millis();
      send_data(0); //Print data for setpoint position to serial port
      send_data(1); //Print data for current position to serial port
      bool new_data = recvWithEndMarker(); //Was a new command recieved? 
      if (new_data){
        break; //Break while loop
      }
    }
  }
  else{ //if no new setpoint, stay put   
    dxlCom.readPresentPosition(servo_id);
    curr_pos = get_result(); //Keep the motor where it is  
  }

  // check for new serial messages  
  recvWithEndMarker();
  
  curr_time = millis();  //keep time
  
  if (data_on){ //Send a continuous stream of data to the serial port
    send_data(0);
    send_data(1);
  }
}
/////////////////////////////////////////////////////////////////////////////////////
/*
 * Conversion functions
 */

/*
 * Convert from motor position to angle
 */
int angle_to_pos(float angle, bool use_degrees){
  if (use_degrees){
    return 512-int((90-angle)*1024/300.0); //Degrees
  }
  else{
    return 512-int((pi/2-angle)*1024/(300.0*pi/180)); //Radians
  }
}
/*
 * Convert from motor position to angle
 */
float pos_to_angle(int pos, bool use_degrees){
  if (use_degrees){
    return 90.0-(512-pos)*300.0/1024.0; //Degrees Dynamixel AX-12A limits are 0 to 300 degrees
  }
  else{
    return pi/2-(512-pos)*(300.0*pi/180)/1024.0; //Radians
  }
}

/*
 * Convert from arrangement space to motor position
 */

int arrange_to_pos(float arrange){
  return 512-int((90.0-arrange*90)*1024.0/300.0);
}

/*
 * Convert from motor position to gripper distance
 */
 
float pos_to_arrange(int pos){
  return 1.00 - (512-pos)*300.0/1024.0/90.00;
}


/*
 * Convert the units from display units to internal units
 */
float convert_units_in(float input){
  switch(units){
    case 0: // If using integer units
      return int(input);
    break;
    case 1: // If using degrees
      return angle_to_pos(input, true);
    break;
    case 2: // If using radians
      return angle_to_pos(input, false);
    break;
    case 3: // If using arrangement space
      return arrange_to_pos(input);
    break;
    default:
      return input;
  }
}
/*
 * Convert the units from internal units to display units
 */
float convert_units_out(float input){
  switch(units){
    case 0: // If using integer units
      return input;
    break;
    case 1: // If using degrees
      return pos_to_angle(input, true);
    break;
    case 2: // If using radians
      return pos_to_angle(input, false);
    break;
    case 3: // If using arrangement space
      return pos_to_arrange(input);
    break;
    default:
      return input;
  }
}
/////////////////////////////////////////////////////////////////////////////////////
/*
 * Print Data
 */
void send_data(int type){
  String out_str =String(curr_time);

  switch (type){
    case 0: //Setpoint
      out_str += '\t';
      out_str += "0";    
      out_str += '\t'+String(setpoint);
      out_str += '\t'+String(convert_units_out(setpoint));    
    break;
    case 1: //Current point
      out_str += '\t';
      out_str += "1";      
      out_str += '\t'+String(curr_pos);
      out_str += '\t'+String(convert_units_out(curr_pos));      
    break;
  }
  Serial.println(out_str);
}

/////////////////////////////////////////////////////////////////////////////////////
/*
Handle Incoming Serial Commands 
*/

String command="";
// Check for new serial data
bool recvWithEndMarker() {
  char endMarker = '\n';
  char rc;  
  while (Serial.available() > 0) {
    rc = Serial.read();
    if (rc != endMarker) {
      command += rc;
    }
    else {
      command.toUpperCase(); //Make the command uppercase to allow users to give any case
      parse_command(command); //Parse command will convert the string command to a motor command
      command="";
      return true;
    }
  }
  return false;
}

// Send a string via serial
void send_string(String bc_string){
  Serial.println(bc_string);
}

// Get one element of a string command with delimeter
String get_string_value(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Parse Commands
/*
 *  This function reads the serial port input and parses to the keyword 
 *  and extracts the correct value and sets the corresponding variable
 */
void parse_command(String command){
  if (command.length()){
    String out_str="_";
    bool echo_one_time=false;
           
    if(command.startsWith("SET")){ //Set the position setpoints in units given (check units before calling)
      if (get_string_value(command,';', 2).length()){ //Set takes two values. 2nd value is setpoint. 1st is for Ctrl-P
        float allset=get_string_value(command,';', 2).toFloat();
        allset = convert_units_in(allset);       
        if (allset<=max_pos & allset>=min_pos){
          setpoint = allset;
        }
        new_setpoint = true;
        out_str+="New ";
      }
      out_str+="SET: ";    
      out_str += '\t'+String(convert_units_out(setpoint));          
    }
    else if(command.startsWith("SPEED")){  //Set the speed of the servos (in direct units)
      if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();       
        speed = allset;        
        out_str+="New ";
      }
      out_str+="SPEED: ";      
      dxlCom.setMovingSpeed(servo_id,speed);        
      out_str += '\t'+String(speed); 
    }
    else if(command.startsWith("TORQUE")){ //Set the torque of the servos (in direct units)     
      if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();        
        torque = allset;        
        out_str+="New ";
      }
      out_str+="TORQUE: ";      
      dxlCom.setTorqueLimit(servo_id,torque);   
      out_str += '\t'+String(torque);
    }
    else if(command.startsWith("CONT")){ //Set the continuous operation mode (i.e. torque-determined stop vs. continuous torque)     
      if (get_string_value(command,';', 1).length()){
        bool allset=bool(get_string_value(command,';', 1).toInt()); 
        cont_hold = allset;        
        out_str+="New ";
      }
      out_str+="CONT: ";      
      out_str += '\t'+String(cont_hold);    
    }
    else if(command.startsWith("MAX")){ //Set the maximum position of the servos (in direct units)      
      if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();
        max_pos = allset;     
        out_str+="New ";
      }
      out_str+="MAX: ";      
      out_str += '\t'+String(max_pos);
    }
    else if(command.startsWith("MIN")){ //Set the maximum position of the servos (in direct units)    
      if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();      
        min_pos = allset;     
        out_str+="New ";
      }
      out_str+="MIN: ";      
      out_str += '\t'+String(min_pos);    
    }
    else if(command.startsWith("ON")){ //Turn on live data output
      data_on = true;
      out_str+="ON: "; 
    }
    else if(command.startsWith("OFF")){ //Turn off live data output
      data_on = false;
      out_str+="OFF: "; 
    } 
    else if(command.startsWith("UNITS")){ //Set the output units
      if (get_string_value(command,';', 1).length()){
        units = constrain(get_string_value(command,';', 1).toInt(),0,4);
        out_str+="New ";
      }
      out_str+="UNITS: " + String(units); 
    }  
    else if(command.startsWith("ECHO")){ //Set the state of command echos
      if (get_string_value(command,';', 1).length()){
        echo_global = bool(get_string_value(command,';', 1).toInt());
        echo_one_time = true;
        out_str+="New ";
      }
      out_str+="ECHO: " + String(echo_global);
    }
    else if(command.startsWith("REBOOT")){ //Reboot a servo
      if (get_string_value(command,';', 1).length()){
        int id = get_string_value(command,';', 1).toInt();
        dxlCom.reboot(id);       
        out_str += "REBOOT: ";
        out_str += '\t'+String(id);
      }
    }
    else if(command.startsWith("PING")){ //Ping a servo
      if (get_string_value(command,';', 1).length()){
        int id = get_string_value(command,';', 1).toInt();
        dxlCom.ping(id);
        out_str += "PING: ";
        out_str += '\t'+String(id);
        out_str += '\t'+String(get_result());
      }
    }
    else if(command.startsWith("VOLT")){ //Get the voltage (in V) of all servos
      out_str+="VOLT: ";      
      dxlCom.readVoltage(servo_id);
      out_str += '\t'+String(get_result()/10.0,1);         
    }
    else if(command.startsWith("TEMP")){ //Get the temperature (in C) of all servos
      out_str+="TEMP: ";      
      dxlCom.readTemperature(servo_id);
      out_str += '\t'+String(get_result());      
    }
    else if(command.startsWith("FIRMWARE")){ //Get the firmware number of all servos
      out_str+="FIRMWARE: ";      
      dxlCom.readFirmware(servo_id);
      out_str += '\t'+String(get_result());
      
    }
    else if(command.startsWith("MODEL")){ //Get the model number of all servos
      out_str+="MODEL: ";      
      dxlCom.readModelNumber(servo_id);
      out_str += '\t'+String(get_result());    
    }
    else if(command.startsWith("LED")){ //Set the LED state of a servo
      if (get_string_value(command,';', 2).length()){
        int id = get_string_value(command,';', 1).toInt();
        bool val = bool(get_string_value(command,';', 2).toInt());
        dxlCom.setLedEnable(id, val);        
      }  
    }   
    else{
      out_str = "Unrecognized Command: ";  
      out_str += command;
    }
    if (echo_global or echo_one_time){
      send_string(out_str);
      echo_one_time = false;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////
/*
 * Functions to Obtain Results from Motor
 */
int get_result(){ //This function returns the result of the last data call and also checks the error
  while(!dxlCom.dxlDataReady());    
  error = dxlCom.readDxlError();
  if(error!=DXL_ERR_SUCCESS) // readDxlResult should always be called before readDxlData
    printDxlError(error);
  return  dxlCom.readDxlResult();
}
void clear_result(){ //This functions resets the motor to prepare for new information by reading error and result
  dxlCom.readDxlError();
  dxlCom.readDxlResult();
}
/////////////////////////////////////////////////////////////////////////////////////
/*
 * Dynamixel Library Functions
 */
void printDxlResult()
{
   while(!dxlCom.dxlDataReady());        // waiting the answer of servo
   printDxlError(dxlCom.readDxlError());
   Serial.println(dxlCom.readDxlResult());
}
void printServoId(String msg)
{
  Serial.print(msg);
  Serial.print(" servo ID ");
  Serial.print(_id);
  Serial.print(" - ");
}
void printDxlError(unsigned short dxlError)
{
  // after any operation error can be retrieve using dx::readDxlResult() (i.e. after read or write operation)
  if(dxlError == DXL_ERR_SUCCESS)
    Serial.println("OK");
  else
  {
    if (dxlError & DXL_ERR_VOLTAGE)
      Serial.print("voltage out of range-");
    if (dxlError & DXL_ERR_ANGLE)
      Serial.print("angle out of range-");
    if (dxlError & DXL_ERR_OVERHEATING)
      Serial.print("overheating-");
    if (dxlError & DXL_ERR_RANGE)
      Serial.print("cmd out of range-");
    if (dxlError & DXL_ERR_TX_CHECKSUM)
      Serial.print("Tx CRC invalid-");
    if (dxlError & DXL_ERR_OVERLOAD )
      Serial.print("overload-");
    if (dxlError & DXL_ERR_INSTRUCTION )
      Serial.print("undefined instruction-");
    if (dxlError & DXL_ERR_TX_FAIL )
      Serial.print("Tx No header-");
    if (dxlError & DXL_ERR_RX_FAIL )
      Serial.print("Rx No header-");
    if (dxlError & DXL_ERR_TX_ERROR  )
      Serial.print("Tx error-");
    if (dxlError & DXL_ERR_RX_LENGTH   )
      Serial.print("Rx length invalid-");  // Not implemented yet
    if (dxlError & DXL_ERR_RX_TIMEOUT)
      Serial.print("timeout-");
    if (dxlError & DXL_ERR_RX_CORRUPT)
      Serial.print("Rx CRC invalid-");
    if (dxlError & DXL_ERR_ID )
      Serial.print("Wrong ID answered-"); // ?? Hardware issue
    Serial.println();
  }
}
