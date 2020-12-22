#include "CommObd2CAN.h"
#include "BoardInterface.h"
#include "LiveData.h"
#include <mcp_CAN.h>

//#include <string.h>

/**
   Connect CAN adapter
*/
void CommObd2Can::connectDevice() {

  Serial.println("CAN connectDevice");

  //CAN = new MCP_CAN(pinCanCs); // todo: remove if smart pointer is ok
  CAN.reset(new MCP_CAN(pinCanCs)); // smart pointer so it's automatically cleaned when out of context and also free to re-init
  if (CAN == nullptr) {
    Serial.println("Error: Not enough memory to instantiate CAN class");
    Serial.println("init_can() failed");
    return;
  }

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    board->displayMessage(" > CAN init OK", "");
  } else {
    Serial.println("Error Initializing MCP2515...");
    board->displayMessage(" > CAN init failed", "");
    return;
  }

  if (liveData->settings.carType == CAR_BMW_I3_2014) {
    //initialise mask and filter to allow only receipt of 0x7xx CAN IDs
    CAN->init_Mask(0, 0, 0x07000000);              // Init first mask...
    CAN->init_Mask(1, 0, 0x07000000);              // Init second mask...
    for (uint8_t i = 0; i < 6; ++i) {
      CAN->init_Filt(i, 0, 0x06000000);           //Init filters
    }
  }

  if (MCP2515_OK != CAN->setMode(MCP_NORMAL)) {  // Set operation mode to normal so the MCP2515 sends acks to received data.
    Serial.println("Error: CAN->setMode(MCP_NORMAL) failed");
    board->displayMessage(" > CAN init failed", "");
    return;
  }

  pinMode(pinCanInt, INPUT);                    // Configuring pin for /INT input

  // Serve first command (ATZ)
  doNextQueueCommand();

  Serial.println("init_can() done");
}

/**
   Disconnect device
*/
void CommObd2Can::disconnectDevice() {

  Serial.println("COMM disconnectDevice");
}

/**
   Scan device list, from menu
*/
void CommObd2Can::scanDevices() {

  Serial.println("COMM scanDevices");
}

/**
   Main loop
*/
void CommObd2Can::mainLoop() {

  CommInterface::mainLoop();

  // Read data
  const uint8_t firstByte = receivePID();
  if ((firstByte & 0xf0) == 0x10) { // First frame, request another
    sendFlowControlFrame();
    delay(10);
    for (uint16_t i = 0; i < 1000; i++) {
      receivePID();
      if (rxRemaining <= 2)
        break;
      delay(1);
      // apply timeout for next frames loop too
      if (lastDataSent != 0 && (unsigned long)(millis() - lastDataSent) > 100) {
        Serial.print("CAN execution timeout (multiframe message).");
        break;
      }
    }
    // Process incomplette messages
    if (liveData->responseRowMerged.length() > 7) {
      processMergedResponse();
      return;
    }
  }
  if (lastDataSent != 0 && (unsigned long)(millis() - lastDataSent) > 100) {
    Serial.print("CAN execution timeout. Continue with next command.");
    liveData->canSendNextAtCommand = true;
    return;
  }
}

/**
   Send command to CAN bus
*/
void CommObd2Can::executeCommand(String cmd) {

  Serial.print("executeCommand ");
  Serial.println(cmd);

  if (cmd.equals("") || cmd.startsWith("AT")) { // skip AT commands as not used by direct CAN connection
    lastDataSent = 0;
    liveData->canSendNextAtCommand = true;
    return;
  }

  // Send command
  liveData->responseRowMerged = "";
  liveData->currentAtshRequest.replace(" ", ""); // remove possible spaces
  String atsh = "0" + liveData->currentAtshRequest.substring(4); // remove ATSH
  cmd.replace(" ", ""); // remove possible spaces
  sendPID(liveData->hexToDec(atsh, 2, false), cmd);
  delay(40);
}

/**
   Send PID
   remark: parameter cmd as const reference to aviod copying
*/
void CommObd2Can::sendPID(const uint16_t pid, const String& cmd) {

  uint8_t txBuf[8] = { 0 }; // init with zeroes
  String tmpStr;

  if (liveData->settings.carType == CAR_BMW_I3_2014)
  {
    struct Packet_t
    {
      uint8_t startChar;
      uint8_t length;
      uint8_t data[6];
    };
    
    Packet_t* pPacket = (Packet_t*)txBuf;
    
    pPacket->startChar = 0x07;
    pPacket->length = cmd.length() / 2;
    
    for (uint8_t i = 0; i < sizeof(pPacket->data); i++) {
      tmpStr = cmd;
      tmpStr = tmpStr.substring(i * 2, ((i + 1) * 2));
      if (tmpStr != "") {
        pPacket->data[i] = liveData->hexToDec(tmpStr, 1, false);
      }
    }
  }
  else
  {
    txBuf[0] = cmd.length() / 2;
    
    for (uint8_t i = 0; i < 7; i++) {
      tmpStr = cmd;
      tmpStr = tmpStr.substring(i * 2, ((i + 1) * 2));
      if (tmpStr != "") {
        txBuf[i + 1] = liveData->hexToDec(tmpStr, 1, false);
      }
    }
  }

  lastPid = pid;
  const uint8_t sndStat = CAN->sendMsgBuf(pid, 0, 8, txBuf); // 11 bit
  //  uint8_t sndStat = CAN->sendMsgBuf(0x7e4, 1, 8, tmp); // 29 bit extended frame
  if (sndStat == CAN_OK)  {
    Serial.print("SENT ");
    lastDataSent = millis();
  }   else {
    Serial.print("Error sending PID ");
  }
  Serial.print(pid);
  for (uint8_t i = 0; i < 8; i++) {
    sprintf(msgString, " 0x%.2X", txBuf[i]);
    Serial.print(msgString);
  }
  Serial.println("");
}

/**
   sendFlowControlFrame
*/
void CommObd2Can::sendFlowControlFrame() {

  uint8_t txBuf[8] = { 0x30, requestFramesCount /*request count*/, 14 /*ms between frames*/ , 0, 0, 0, 0, 0 };
  
  // insert 0x07 into beginning for BMW i3
  if (liveData->settings.carType == CAR_BMW_I3_2014) {
    memmove(txBuf + 1, txBuf, 7);
    txBuf[0] = 0x07;
  }
    
  const uint8_t sndStat = CAN->sendMsgBuf(lastPid, 0, 8, txBuf); // 11 bit
  if (sndStat == CAN_OK)  {
    Serial.print("Flow control frame sent ");
  }   else {
    Serial.print("Error sending flow control frame ");
  }
  Serial.print(lastPid);
  for (auto txByte : txBuf) {
    sprintf(msgString, " 0x%.2X", txByte);
    Serial.print(msgString);
  }
  Serial.println("");
}

//static void mockupReceiveCanBuf(INT32U *id, INT8U *len, INT8U buf[])
//{
//  static uint8_t counter = 0;
//  
//  std::unordered_map<uint8_t, std::vector<uint8_t>> packets = {
//    { 0, { 0xF1, 0x05, 0x62, 0xDD, 0xB4, 0x92, 0xC2 } },
//    { 1, { 0xF1, 0x10, 34, 0xDD, 0xB4, 0x92, 0xC2 } },
//    { 2, { 0xF1, 0x21, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4 } },
//    { 3, { 0xF1, 0x22, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4 } },
//    { 4, { 0xF1, 0x23, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4 } },
//    { 5, { 0xF1, 0x24, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4 } },
//    { 6, { 0xF1, 0x25, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4 } },
//    { 7, { 0xF1, 0x26, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4 } }
//  };
//  
//  if (counter >= packets.size()) 
//    counter = 0;
//  
//  *id = 0x607;
//  *len = packets[counter].size();
//  //memset(buf, 0, 7);
//  
//  
//  memcpy(buf, packets[counter].data(), 7);
//  
//  counter++;
//}

/**
   Receive PID
*/
uint8_t CommObd2Can::receivePID() {
  
  if (!digitalRead(pinCanInt))                        // If CAN0_INT pin is low, read receive buffer
  {
    lastDataSent = millis();
    Serial.print(" CAN READ ");
    CAN->readMsgBuf(&rxId, &rxLen, rxBuf);      // Read data: len = data length, buf = data byte(s)
//    mockupReceiveCanBuf(&rxId, &rxLen, rxBuf);
    
    if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), rxLen);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, rxLen);

    Serial.print(msgString);

    if ((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (uint8_t i = 0; i < rxLen; i++) {
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }

    if (rxLen == 5) {
      Serial.println(" [Ignoring 5 bytes long packet]");
      return 0xff;
    }
    
    Serial.println();
    processFrameBytes();
    //processFrame();
  } else {
    //Serial.println(" CAN NOT READ ");
    return 0xff;
  }

  return rxBuf[0 + liveData->rxBuffOffset];
}

static void printHexBuffer(uint8_t* pData, const uint16_t length, const bool bAddNewLine)
{
  char str[8] = { 0 };
  
  for (uint8_t i = 0; i < length; i++) {
    sprintf(str, " 0x%.2X", pData[i]);
    Serial.print(str);
  }
  
  if (bAddNewLine) {
    Serial.println();
  }
}

static void buffer2string(String& out_targetString, uint8_t* in_pBuffer, const uint16_t in_length)
{
  char str[8] = { 0 };
  
  for (uint16_t i = 0; i < in_length; i++) {
    sprintf(str, "%.2X", in_pBuffer[i]);
    out_targetString += str;  
  }
  
  
}

/**
   Process can frame on byte level
 */
bool CommObd2Can::processFrameBytes() {
  
  uint8_t* pDataStart = rxBuf + liveData->rxBuffOffset; // set pointer to data start based on specific offset of car
  const uint8_t frameType = (*pDataStart & 0xf0) >> 4;
  const uint8_t frameLenght = rxLen - liveData->rxBuffOffset;
  
  switch (frameType) {
  case 0: // Single frame
    {
      struct SingleFrame_t
      {
        uint8_t size : 4;
        uint8_t frameType : 4;
        uint8_t pData[];
      };
      
      SingleFrame_t* pSingleFrame = (SingleFrame_t*)pDataStart;
      mergedData.assign(pSingleFrame->pData, pSingleFrame->pData + pSingleFrame->size);
      
      rxRemaining = 0;
      
      Serial.print("---Processing SingleFrame payload: "); printHexBuffer(pSingleFrame->pData, pSingleFrame->size, true);
    }
    break;
    
  case 1: // First frame
    {
      struct FirstFrame_t
      {
        uint8_t sizeMSB : 4;
        uint8_t frameType : 4;
        uint8_t sizeLSB : 8;
        uint8_t pData[];
    
        uint16_t lengthOfFullPacket() { return (256 * sizeMSB) + sizeLSB; }
    
      };
      
      FirstFrame_t* pFirstFrame = (FirstFrame_t*)pDataStart;
      
      rxRemaining = pFirstFrame->lengthOfFullPacket(); // length of complete data
      
      mergedData.clear();
      dataRows.clear();
      
      const uint8_t framePayloadSize = frameLenght - sizeof(FirstFrame_t);    // remove one byte of header
      dataRows[0].assign(pFirstFrame->pData, pFirstFrame->pData + framePayloadSize);
      rxRemaining -= framePayloadSize;
      
      Serial.print("---Processing FirstFrame payload: "); printHexBuffer(pFirstFrame->pData, framePayloadSize, true);
    }
    break;
    
  case 2: // Consecutive frame
    {
      struct ConsecutiveFrame_t
      {
        uint8_t index : 4;
        uint8_t frameType : 4;
        uint8_t pData[];
      };
      
      const uint8_t structSize = sizeof(ConsecutiveFrame_t);
      Serial.print("[debug] sizeof(ConsecutiveFrame_t) is expected to be 1 and it's "); Serial.println(structSize);
      
      ConsecutiveFrame_t* pConseqFrame = (ConsecutiveFrame_t*)pDataStart;
      const uint8_t framePayloadSize = frameLenght - sizeof(ConsecutiveFrame_t);  // remove one byte of header
      dataRows[pConseqFrame->index].assign(pConseqFrame->pData, pConseqFrame->pData + framePayloadSize);
      rxRemaining -= framePayloadSize;
      
      Serial.print("---Processing ConsecFrame payload: "); printHexBuffer(pConseqFrame->pData, framePayloadSize, true);
    }
    break;
    
  default:
    Serial.print("Unknown frame type within CommObd2Can::processFrameBytes(): "); Serial.println(frameType);
    return false;
    break;
  }
  
  
  if (frameType == 0)
  {
    // single frame - process directly
    buffer2string(liveData->responseRowMerged, mergedData.data(), mergedData.size());
    liveData->vResponseRowMerged.assign(mergedData.begin(), mergedData.end());
    processMergedResponse();
  }
  else if (rxRemaining <= 0)
  {
    // multiple frames and no data remaining - merge everything to single packet
    //for(const auto& row : dataRows)
    for (int i = 0; i < dataRows.size(); i++)
    {
      Serial.print("---merging packet index ");
      Serial.print(i);
      Serial.print(" with length ");
      Serial.println(dataRows[i].size());
      
      mergedData.insert(mergedData.end(), dataRows[i].begin(), dataRows[i].end());
      
    }
    
    buffer2string(liveData->responseRowMerged, mergedData.data(), mergedData.size());
    liveData->vResponseRowMerged.assign(mergedData.begin(), mergedData.end());
    processMergedResponse();
    
  }
  
  return true;
}

/**
   Process can frame
   https://en.wikipedia.org/wiki/ISO_15765-2
*/
bool CommObd2Can::processFrame() {

  const uint8_t frameType = (rxBuf[0] & 0xf0) >> 4;
  uint8_t start = 1; // Single and Consecutive starts with pos 1
  uint8_t index = 0; // 0 - f

  liveData->responseRow = "";
  switch (frameType) {
    // Single frame
    case 0:
      rxRemaining = (rxBuf[1] & 0x0f);
      requestFramesCount = 0;
      break;
    // First frame
    case 1:
      rxRemaining = ((rxBuf[0] & 0x0f) << 8) + rxBuf[1];
      requestFramesCount = ceil((rxRemaining - 6) / 7.0);
      liveData->responseRowMerged = "";
      for (uint16_t i = 0; i < rxRemaining - 1; i++)
        liveData->responseRowMerged += "00";
      liveData->responseRow = "0:";
      start = 2;
      break;
    // Consecutive frames
    case 2:
      index = (rxBuf[0] & 0x0f);
      sprintf(msgString, "%.1X:", index);
      liveData->responseRow = msgString; // convert 0..15 to ascii 0..F);
      break;
  }

  Serial.print("> frametype:");
  Serial.print(frameType);
  Serial.print(", r: ");
  Serial.print(rxRemaining);
  Serial.print("   ");

  for (uint8_t i = start; i < rxLen; i++) {
    sprintf(msgString, "%.2X", rxBuf[i]);
    liveData->responseRow += msgString;
    rxRemaining--;
  }

  Serial.print(", r: ");
  Serial.print(rxRemaining);
  Serial.println("   ");

  //parseResponse();
  // We need to sort frames
  // 1 frame data
  Serial.println(liveData->responseRow);
  // Merge frames 0:xxxx 1:yyyy 2:zzzz to single response xxxxyyyyzzzz string
  if (liveData->responseRow.length() >= 2 && liveData->responseRow.charAt(1) == ':') {
    //liveData->responseRowMerged += liveData->responseRow.substring(2);
    uint8_t rowNo = liveData->hexToDec(liveData->responseRow.substring(0, 1), 1, false);
    uint16_t startPos = (rowNo * 14) - ((rowNo > 0) ? 2 : 0);
    uint16_t endPos = ((rowNo + 1) * 14) - ((rowNo > 0) ? 2 : 0);
    liveData->responseRowMerged = liveData->responseRowMerged.substring(0, startPos) + liveData->responseRow.substring(2) + liveData->responseRowMerged.substring(endPos);
    Serial.println(liveData->responseRowMerged);
  }

  // Send response to board module
  if (rxRemaining <= 2) {
    processMergedResponse();
    return false;
  }

  return true;
}

/**
   processMergedResponse
*/
void CommObd2Can::processMergedResponse() {
  Serial.print("merged:");
  Serial.println(liveData->responseRowMerged);
  board->parseRowMerged();
  liveData->responseRowMerged = "";
  liveData->canSendNextAtCommand = true;
}
