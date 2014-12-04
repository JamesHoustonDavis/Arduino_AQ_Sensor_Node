////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Arduino sensor node code written for Professors Eglash & Sawyer of RPI

This code establishes a network of arduino nodes. Each node may have a sensor that takes and stores data.
Each node may communicate with all other adjacent nodes and communicate data across the network. It may
also interface with any phone running the arduino - android connection app.

The code runs on 3 interrupts, 2 for communication 1 for sleeping / data collection. At communication a
message can be sent with header test, request, information (as required), request...... \n (EOM). It is
important that the communication not occur at the same time as a wakeup cycle.

To setup the network you must first, from base station, request FindAdjacentNodes, then GenerateRoutingTable,
then SynchronizeNeighbors, and finally ResetNetwork.

*/
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SoftwareSerial.h> 
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

//settables
const byte StationID = 2;
const int lengthOfDataToSend = 100;
const byte AcceptableNumberOfConnectionAttempts = 10; //
const unsigned int NumberOfAcceptableIdleCyclesBeforeSleeping = 100000; //just random trial and error
const unsigned long MilliSecondsToSleepBetweenDataCollectionCycles = 24000; //20 minutes = 12000000

//data
byte data[lengthOfDataToSend];
byte power;
String MACAddress = "";
unsigned long sleepUntilTime = 0;

//communications counters and booleans
int points = 0;
int replyCount = 0;
byte buff[] = {0,0,0,0};
unsigned long messageTime = 0;
char reply;
String messageFromBluetooth = "";
String messageFromSerial = "";
String messageFromColorSenosr = "";
unsigned int numberOfIdleCycles = 0;
int messagesReceived = 0;

//MessageCodes
const char RequestData = 'R'; //request the data of an individual node
const char BaseStationRequest = 'B'; //request all data from the network
const char RequestPower = 'P'; //request the power from the node
const char ReplyPower = 'L'; //incoming power data (1byte data)
const char ReplyData = 'D'; //variable length
const char SendingForwardingID = 'F'; //1byte incoming
const char SendingReturnID = 'I'; //16 char mac address to allow for base station
const char ResetNetwork = 'N'; //reset the network data & states
const char TestingConnection = 'T'; //test to see if the connection is valid (sleeping nodes sometimes miss the first char)
const char CompletedBaseStationRequest = 'C'; //this branch of the network data request is completed
const char SynchronizeNeighbors = 'S'; //synchronize the time of all your neighbors
const char IncomingTimeMessage = '1'; //’the time is currently’
const char RequestTimeMessage = '2'; //’please send me the time where you are’
const char TimeDifferenceOutgoingMessage = '3'; //’the time between are clocks is’
const char DoneTimeSync = '4'; //this branch of the network has had it’s time synchronized
const char FindAdjacentNodes = 'A'; //search the nearby nodes
const char GenerateRoutingTable = 'G'; //search the nearby nodes, but exclude the list of nodes variable byte
const char IncomingListOfNodes = 'O'; //exclude all these from your list
const char RequestCloseConnection = 'K'; //Because of android and it's stupid tunnels
const char RequestFireYourLasers = 'E'; //Fires The Lasers
const char FindYourMACAddress = 'M'; //remove after testing
const char AssignYourName = 'Q'; //remove after testing
const char FindName = '9'; //remove after testing

//states
const char SearchingForNodes = 'S';
const char ConnectingToNode = 'C';
const char IdleState = 'N';
const char WaitingForMACAddress = 'W';
const char WaitToExitCommandMode = 'E'; //you can only exit once you have finished receiving :(
char state = IdleState;
boolean retryNodeSearch = false;
boolean hasReceivedBaseStationRequest = false;
boolean hasSynchronizedBaseStation = false;
boolean isLasersOn = false;
boolean isNewRoutingTable = false;

unsigned int cyclesOfIdle = 0;

//Information on relay data
char relayMessage = IdleState;
byte relayNode = 0;
String messageFromSearchingForOtherStations = "";
String returnRelayMessageTo = ""; //macaddress, because it must also allow for phones
int relayMessageTo; //relay station number
signed long timeSyncIncoming = 0; //time difference of the incoming message
signed long timeSyncOutgoing = 0; // time difference of the outgoing message
signed long delayOfNodeToBaseStation = 0;
int connectionAttempts = 0;
byte retrySearchCount = 0;

//Information on network structure
const String NetworkNodeNamePrefix = "AQSN";
String routingTable[] = {"00A","00A","00A","00A","00A"};
int numberOfAdjacentNodes = 0;
int numberOfAdjacentNodesContacted = 0;
String adjacentNodes[] = {"00A","00A","00A","00A","00A"};
byte alreadyFoundNodes[255];
byte numberOfAlreadyFoundNodes = 0;

// bluetooth serial
int bluetoothTx = 14;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D3
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// color sensor

int colorSensorTx = 4;  // TX-O pin of bluetooth mate, Arduino D2
int colorSensorRx = 6;  // RX-I pin of bluetooth mate, Arduino D3
SoftwareSerial colorSensor(colorSensorTx, colorSensorRx);


//programming test cases
int RXLED = 17;
boolean SendStraightToSerial = false;

//watch dog timer
volatile int f_wdt=0;
extern volatile unsigned long timer0_millis;
int time = 0;

void setup()
{
  Serial.begin(2400);  // Begin the serial monitor at 9600bps
  colorSensor.begin(38400);
  pinMode(16,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(3,OUTPUT);
  //serial and bluetooth interrupts (wakes up upon command)
  attachInterrupt(0,LowBatteryInterruptServiceRoutine,CHANGE);
  attachInterrupt(1,BluetoothInterruptServiceRoutine,CHANGE);
  
  //watch dog timer settup (samples data on command)
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 1<<WDP3; // 8.0 seconds
  WDTCSR |= _BV(WDIE);
  //WDE &= 0;
  //WDTCSR &= B11110111;
  BluetoothInitialization();
  ClearMessages();  
  ExitCommandMode();
  ClearMessages();
  FindOwnMACAddress();
  sleepUntilTime = points*MilliSecondsToSleepBetweenDataCollectionCycles-delayOfNodeToBaseStation;
}

void loop()
{
  //digitalWrite(16,HIGH);
  ReceiveDataFromBluetooth();
  ReceiveDataFromSerial();
  ReceiveDataFromColorSensor();
  
  if(f_wdt == 1)//go to sleep
  {
    f_wdt = 0;
    sleepUntilTime = points*MilliSecondsToSleepBetweenDataCollectionCycles-delayOfNodeToBaseStation;
    while(millis()<sleepUntilTime)
    {
      delay(30);
      enterSleep();
      IncrementMillis(8000);//8 seconds passed while sleeping
      if(bluetooth.available()||Serial.available()) // don't sleep if there is communication
        break;
    }
    if(millis()>sleepUntilTime)
    {
      f_wdt = 1;
    }
  }
  if(millis()>sleepUntilTime) // if it's data collection time
  {
    Serial.println("data collected");
    colorSensor.println('R');
    data[points] = analogRead(A0);
    points++;
    points = points % lengthOfDataToSend;
    sleepUntilTime = points*MilliSecondsToSleepBetweenDataCollectionCycles-delayOfNodeToBaseStation;
  }
  numberOfIdleCycles++;
  if(numberOfIdleCycles >= NumberOfAcceptableIdleCyclesBeforeSleeping) // if you're idle
  {
    f_wdt = 1; //go to sleep next cycle
    
    numberOfIdleCycles=0;
  }
}

void ReceiveDataFromSerial()
{
  delay(1);
  if(Serial.available())  // If stuff was typed in the serial monitor
  {
    reply = (char)Serial.read();
    //bluetooth.print(reply);
    //Serial.print(reply);
    
    if(reply == '\n') //if end of message
    {
      if(!SendStraightToSerial)
      {
          ReceiveMessage(messageFromSerial);
          Serial.println("Command: " +messageFromSerial);
      }
      else 
        {
          bluetooth.print(messageFromSerial);
          Serial.println("Direct: " +messageFromSerial);
      
        }
      messageFromSerial = "";
    }
    else if (reply == '!')
    {
      SendStraightToSerial = !SendStraightToSerial;
    }
    else if (reply == '\\')
    {
      if(SendStraightToSerial)
      {
          bluetooth.print(messageFromSerial+'\r');
          Serial.println("Direct: " +messageFromSerial);
          messageFromSerial = "";
      }
    }
    else
    {
      messageFromSerial += reply;
    }
    
    numberOfIdleCycles = 0;
  }
}

void ReceiveDataFromBluetooth()
{
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    reply = (char)bluetooth.read();
    Serial.print(reply);
    if(reply == '\n') //if end of message
    {
      ReceiveMessage(messageFromBluetooth);
      //Serial.println(messageFromBluetooth);
      messageFromBluetooth = "";
    }
    else
    {
      messageFromBluetooth += reply;
    }
    numberOfIdleCycles = 0;
  }
}

void ReceiveDataFromColorSensor()
{
  if(colorSensor.available())  // If the bluetooth sent any characters
  {
    reply = (char)colorSensor.read();
    Serial.print(reply);
    if(reply == '\n') //if end of message
    {
      Serial.println(messageFromColorSenosr);
      messageFromColorSenosr = "";
    }
    else
    {
      messageFromColorSenosr += reply;
    }
    numberOfIdleCycles = 0;
  }
}

void enterSleep()
{
  numberOfIdleCycles = 0;
  attachInterrupt(0,LowBatteryInterruptServiceRoutine,CHANGE);
  attachInterrupt(1,BluetoothInterruptServiceRoutine,CHANGE);
  digitalWrite(RXLED, HIGH);
  TXLED0;
  Serial.println("sleeping");
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();
  wdt_reset();//make sure it's 8 seconds
  sleep_mode();
  sleep_disable();
  power_all_enable();
}

void BluetoothInitialization()
{
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)bluetooth.read());  
  }
  bluetooth.println("U,2400,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)bluetooth.read());  
  }
  bluetooth.begin(2400);  // Start bluetooth serial at 9600
  ExitCommandMode();
  SetBluetoothName();
}

ISR(WDT_vect) //the watch dog interrupt service routine... it has to have this specific name.
{
  //this has to be here so that it goes to the ISR instead of rebooting.
}

void LowBatteryInterruptServiceRoutine()
{
  if(digitalRead(3)==1) // if there is power
  {
    //wdt_enable(); // turn on watch dog
  }
  else // if there isn't much power
  {
    //wdt_disable(); // turn off watch dog
  }
  
}

void BluetoothInterruptServiceRoutine()
{
  detachInterrupt(1);
}
  String _MAC, name, _ID;
void ReceiveMessage(String message)
{
  message.trim();
  delay(2);
  //Serial.println(message);
  switch(state)
  {
    case SearchingForNodes:
      if(TrySplitSearchReplyIntoName(message,_MAC, name, _ID))
      {
        name.trim();
        if(name == "")
        {
          retryNodeSearch = true;
          Serial.println("Retry because of message: ");
          delay(10);
        }
        else if(name.substring(0,4) == NetworkNodeNamePrefix)
        {
          byte contactedStationID = 0;
          Serial.println("Found neighbor: " + name);
          contactedStationID = byte(message.substring(4,7).toInt());
          if(!contains(alreadyFoundNodes, contactedStationID, numberOfAlreadyFoundNodes))
          {
            adjacentNodes[numberOfAdjacentNodes] = _MAC;
            alreadyFoundNodes[NumberOfAlreadyFoundNodes] = contactedStationID;
            numberOfAdjacentNodes++;
            NumberOfAlreadyFoundNodes++;
            
          }
        }
      }
      else if(message.substring(0,12)=="Inquiry Done")
      {
        if(!retryNodeSearch && retrySearchCount < 6)
        {
          Serial.println("Search Completed");
          Serial.println("Number of adjacent Nodes: " + String(numberOfAdjacentNodes));
          state = IdleState;
          ExitCommandMode();
          return; 
        }
        else
        {
          retrySearchCount++;
          SearchForNodes();
          retryNodeSearch = false;
          numberOfAdjacentNodes = 0;
        }
      }
    break;
    case ConnectingToNode:
        state = IdleState;
    break;
    case WaitingForMACAddress:
    if(message.length()>4)
    {
       MACAddress=message; 
       Serial.println("Found MAC Address");
       Serial.println(MACAddress);
       state=IdleState;
       ExitCommandMode();
       EnterCommandMode();
       SearchForNodes();
    }
    break;
    case IdleState://ready to accept commands / service connections
      if(!message.endsWith("."))   return;// all commands end with period
      HandleRequests(message);
    break;
  }
}

void HandleRequests(String message)
{
  for(int i = 0; i<message.length(); i++)
  {
    byte tempPower;
    reply = message.charAt(i);
    switch(reply)
    {
          case RequestData:
            Serial.println("data requested");
            if(relayMessageTo==StationID) // if the request is directed to you
            {
              SendDataToConnection();
            }
            else
            {
             KillConnection();
             ConnectToNode(routingTable[relayMessageTo]);
             SendRelayMessageHeader();
             bluetooth.print(RequestData);
             EndMessage();
            }
          break;
          case BaseStationRequest: //requesting all data from the network
            if(!hasReceivedBaseStationRequest)
            {
              hasReceivedBaseStationRequest = true;
              Serial.println("data of entire network requested");
              delay(5);
              SendDataToConnection();
              numberOfAdjacentNodesContacted = 0;
              if(numberOfAdjacentNodesContacted<numberOfAdjacentNodes) // because sometimes you might have a single node sitting alone with no one to synchronize with
              {
                KillConnection();
                ConnectToNode(adjacentNodes[numberOfAdjacentNodesContacted]);
                SendRelayMessageHeader();
                Serial.println("Forwarding request");
                delay(2);
                bluetooth.print(BaseStationRequest);
                EndMessage();
              }
            }
            else // if you're already doing it, or it's already done, no need to explore this branch
            {
              Serial.println("Replying completed base station request");
              bluetooth.print(CompletedBaseStationRequest);
              EndMessage();
            }
          break;
          case CompletedBaseStationRequest: // all data from adjacent nodes reported
              numberOfAdjacentNodesContacted++;
              Serial.println("Node " + String(numberOfAdjacentNodesContacted) + " replied done");
              KillConnection();
              if(numberOfAdjacentNodesContacted<numberOfAdjacentNodes) // send requests to the other branches
              {
                Serial.println("Continue requesting network data");
                ConnectToNode(adjacentNodes[numberOfAdjacentNodesContacted]);
                bluetooth.print(SendingReturnID);
                bluetooth.print(MACAddress);
                bluetooth.print(BaseStationRequest);
                EndMessage();
              }
              else //if all the branches are done, reply to previous completed base station request
              {
                 Serial.println("All done, letting source know");
                 if(returnRelayMessageTo!="") // because I sometimes request things over serial
                 {
                   ConnectToNode(returnRelayMessageTo);
                   bluetooth.print(CompletedBaseStationRequest);
                   EndMessage();
                 }
              }
          break;
          case RequestPower:
            if(relayMessageTo==StationID) // if the request is directed to you
            {
               power = (byte)analogRead(0);
               bluetooth.print(ReplyPower);
               bluetooth.print(power);
               EndMessage();
            }
            else
            {
             KillConnection();
             ConnectToNode(routingTable[relayMessageTo]);
             SendRelayMessageHeader();
             bluetooth.print(RequestPower);
             EndMessage();
            }
            
          break;
          case ReplyPower:
             tempPower = message.charAt(i+1);
             KillConnection();
             ConnectToNode(returnRelayMessageTo);
             bluetooth.print(ReplyPower);
             bluetooth.print(tempPower);
             EndMessage();
          break;
          case ReplyData:
            Serial.println("Receiving Reply Data");
            for(int j = i+1; j< message.length() && j-i-1<lengthOfDataToSend; j++)
            {
              data[j] = message.charAt(j);
            }
             KillConnection();
             Serial.println("Attempting to connect source node");
               if(returnRelayMessageTo!="") // because I sometimes request things over serial
               {
                 ConnectToNode(returnRelayMessageTo);
                 SendDataToConnection();
               }
          break;
          case SendingForwardingID:
            relayMessageTo = message.substring(i+1,i+4).toInt(); //should be the 3 char station number
            Serial.println("Forward to: " + String(relayMessageTo));
            i+=3;
          break;
          case SendingReturnID:
            if(!hasReceivedBaseStationRequest && !hasSynchronizedBaseStation) // don't let other nodes contacting you about the base station request change the return to base path change
            {
              returnRelayMessageTo = message.substring(i+1,i+1+12);
              Serial.println("Return to: " + returnRelayMessageTo);
            }
            i+=12;
          break;
          case ResetNetwork:
            points = 0;
            hasReceivedBaseStationRequest = false;
            hasSynchronizedBaseStation = false;
            //this needs to propogate outward
          break;
          case IncomingTimeMessage:
            timeSyncIncoming = millis() - delayOfNodeToBaseStation - StringToLong(message.substring(i+1,4)); // new incoming time difference
            Serial.println("received a new time");
            if(hasSynchronizedBaseStation)
            {
              Serial.println("Sending Time Delta");
              bluetooth.print(TimeDifferenceOutgoingMessage);
              bluetooth.print(timeSyncIncoming);
              bluetooth.print('.');
              EndMessage();
              Serial.println("Forwarding Synchronization");
              bluetooth.print(SynchronizeNeighbors);
              EndMessage();
              numberOfAdjacentNodesContacted++;
            }
          break;
          case RequestTimeMessage:
            bluetooth.print(IncomingTimeMessage);
            bluetooth.print(millis() - delayOfNodeToBaseStation); // send it your time
            EndMessage();
          break;
          case TimeDifferenceOutgoingMessage:
            timeSyncOutgoing = StringToLong(message.substring(i+1,4));
            if(!hasSynchronizedBaseStation)delayOfNodeToBaseStation = (timeSyncIncoming - timeSyncOutgoing)/2;  // once
          break;
          case SynchronizeNeighbors:
            if(!hasSynchronizedBaseStation)
            {
                KillConnection();
                numberOfAdjacentNodesContacted = 0;
                if(numberOfAdjacentNodesContacted<numberOfAdjacentNodes) // because sometimes you might have a single node sitting alone with no one to synchronize with
                {
                  hasSynchronizedBaseStation = true;
                  ConnectToNode(adjacentNodes[numberOfAdjacentNodesContacted]);
                  SendRelayMessageHeader();
                  Serial.println("sending current time");
                  bluetooth.print(IncomingTimeMessage);
                  bluetooth.print(millis() - delayOfNodeToBaseStation);
                  EndMessage();
                  Serial.println("Request Time Back");
                  bluetooth.print(RequestTimeMessage);
                  EndMessage();
              }
            }
            else
            {
                bluetooth.print(DoneTimeSync);
                EndMessage();
            }
          break;
          case DoneTimeSync:
              KillConnection();
              numberOfAdjacentNodesContacted++;
              if(numberOfAdjacentNodesContacted<numberOfAdjacentNodes)
              {
                Serial.println("Contact next node");
                ConnectToNode(adjacentNodes[numberOfAdjacentNodesContacted]);
                bluetooth.print(IncomingTimeMessage);
                bluetooth.print(millis() - delayOfNodeToBaseStation);
                EndMessage();
                bluetooth.print(RequestTimeMessage);
                EndMessage();
              }
              else
              {
                Serial.println("Done Synchronizing");
                ConnectToNode(returnRelayMessageTo);
                bluetooth.print(DoneTimeSync);
                EndMessage();
                numberOfAdjacentNodesContacted = 0;
              }
          break; 
          case FindAdjacentNodes:
            numberOfAdjacentNodes = 0;
            KillConnection();
            retrySearchCount = 0;
            EnterCommandMode();
            SearchForNodes();
          break;
          case GenerateRoutingTable:
            isNewRoutingTable =true;
            numberOfAdjacentNodes = 0;
            KillConnection();
            retrySearchCount = 0;
            EnterCommandMode();
            SearchForNodes();
          break;
          case IncomingListOfNodes:
            i++;
            while(i< message.length())
            {
              AlreadyFoundNodes[NumberOfAlreadyFoundNodes] = (byte) message.charAt(i);
              i++;
            }
          break;
          case TestingConnection:
            Serial.println("this is the message that requested new mac address: " + message);
            //FindOwnMACAddress();
          break;
          case AssignYourName:
            SetBluetoothName();
            ExitCommandMode();
          break;
          case FindName:
            FindOwnName();
          break;
          case '.':
            return;
          break;
          case RequestCloseConnection: //only for the android
          break;
          case RequestFireYourLasers: // turn on the lasers
            Serial.println("fire'n my lasers");
            if(isLasersOn)
            {
              TurnOnPump();
            }
            else
            {
             TurnOffPump(); 
            }
            isLasersOn = !isLasersOn;
          break;
          default:
            //Serial.println("error parsing data");
            return;
          break;
    }
  }
}
boolean TrySplitSearchReplyIntoName(String message, String &_MAC, String &name, String &_ID)
{
  _MAC = "";
  name = "";
  _ID = "";
  
  int IndexOfFirstComma = message.indexOf(',');
  if(IndexOfFirstComma>-1)
  {
    int IndexOfSecondComma = message.indexOf(',',IndexOfFirstComma+1);
      if(IndexOfSecondComma > -1)
      {
        _MAC = message.substring(0,IndexOfFirstComma);
        Serial.println("MAC: " + _MAC);
        delay(10);
        name = message.substring(IndexOfFirstComma+1,IndexOfSecondComma);
        _ID = message.substring(IndexOfSecondComma+1);
        return true;
      } 
  }
  return false;
}
void SearchForNodes()
{
  retryNodeSearch = false;
  Serial.println("searching for new neighbors");
  bluetooth.println("I,8");
  state = SearchingForNodes;
}
void ConnectToNode(String NameOfNode)
{
  KillConnection();
  EnterCommandMode();
  connectionAttempts = 0;
  ClearMessages(); // remove any remaining messages
  while(connectionAttempts < AcceptableNumberOfConnectionAttempts) // connect a given number of times
  {
    Serial.println("Connect");
    Serial.println("Connecting to " + NameOfNode);
    delay(10);
    bluetooth.println("C,"+NameOfNode);
    delay(5000);
    ClearMessages(); // remove any remaining messages
    if(IsConnected()) break; //if you don't hear anything back, it means it's actively connected
    Serial.println("Connection failed on character " + (char)bluetooth.read());
    connectionAttempts++;
  }
  ExitCommandMode();
  if(connectionAttempts > AcceptableNumberOfConnectionAttempts) Serial.println("failed to connect");
    
  //state = ConnectingToNode;
}
boolean IsConnected()
{
    int cycles = 0;
    EnterCommandMode();
    delay(100);
    ClearMessages(); // get rid of CMD line
    bluetooth.println("GK"); //send request for connection status
    while(!bluetooth.available() &&cycles <100) //wait for reply or timeout
    {
      delay(30);
      cycles++;
    }
    String message = SynchronislyReceiveDataFromBluetooth();
    message.trim();
    Serial.println("Connection Message: " + message);
    delay(10);
    if(cycles<100)//if not time out
    {
      delay(10);
      if(message == "1")
      {  
        ClearMessages(); // get rid of any further replies
        return true;
      }
      else
      {
        ClearMessages(); // get rid of any further replies
        return false;
      }
    }
    return false;
}
void FindOwnMACAddress()
{
  EnterCommandMode();
  state = WaitingForMACAddress;
  Serial.println("Requesting mac address");
  bluetooth.println("GB");
}
void FindOwnName()
{
  EnterCommandMode();
  state = WaitingForMACAddress;
  Serial.println("Requesting name");
  bluetooth.println("GN");
}
void SetBluetoothName()
{
  EnterCommandMode();
  Serial.println("Setting node name to: " + NetworkNodeNamePrefix + GetThreeDigitStationID());
  bluetooth.println("SN," + NetworkNodeNamePrefix + GetThreeDigitStationID());
  delay(100);
}
void KillConnection()
{
  bluetooth.print(RequestCloseConnection);
  EndMessage();
  delay(10);
  EnterCommandMode();
  bluetooth.println("K,");
  ExitCommandMode();
}
void EnterCommandMode()
{
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(200);  // Short delay, wait for the Mate to send back CMD
}
void ExitCommandMode()
{
  bluetooth.println("---");  // Exit command mode
  delay(100); 
}
void SendRelayMessageHeader()
{
   bluetooth.print(SendingReturnID);
   bluetooth.print(MACAddress);
   bluetooth.print(SendingForwardingID);
   bluetooth.print(relayMessageTo);
   bluetooth.print('.');
   bluetooth.print('\n');
}
long StringToLong(String message)
{
  message.getBytes(buff,4);
  memcpy(&messageTime, buff, sizeof(long));
  return messageTime;
}
void EndMessage()
{
  bluetooth.print('.');
  bluetooth.print('\n');
}
void SendDataToConnection()
{
  bluetooth.print(ReplyData);
  Serial.print(ReplyData);
 for(int i = 0; i< lengthOfDataToSend; i++)
 {
  bluetooth.print(data[i]); 
  Serial.print(data[i]);
 }
 Serial.print('\n');
 bluetooth.print('.');
 bluetooth.print('\n');
 delay(2);
}
void ClearMessages() //warning, using a while loop
{
  delay(10);
  while(bluetooth.available())  // If stuff was typed in the serial monitor
  {
    (char)bluetooth.read();
    delay(2);
  }
}
String GetThreeDigitStationID()
{
  if(StationID <10)
  {
   return ("00" + String(StationID)); 
  }
  else if(StationID<100)
  {
    return ("0" + String(StationID));
  }
  else 
  {
    return ("" + String(StationID));
  }
}
String SynchronislyReceiveDataFromBluetooth()
{
  String message;
  while(bluetooth.available())  // If the bluetooth sent any characters
  {
    reply = (char)bluetooth.read();
    if(reply == '\n') //if end of message
    {
      return message;
    }
    else
    {
      message += reply;
    }
    delay(10);
  }
}
void IncrementMillis(unsigned long new_millis){
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis = millis()+new_millis;
  SREG = oldSREG;
}
void SetMillis(unsigned long new_millis){
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis = new_millis;
  SREG = oldSREG;
}
void TurnOnPump()
{
  digitalWrite(4,HIGH);
  digitalWrite(3,HIGH);
}
void TurnOffPump()
{
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
}
boolean contains(byte[] array, byte val, int length)
{
  for(int i = 0; i< length; i++)
  {
    if(array[i] == val)
    {
     return true;
    } 
  }
  return false;
}
