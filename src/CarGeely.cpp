/*
      eNiro/Kona chargings limits depending on battery temperature (min.value of 01-04 battery module)
  >= 35°C BMS allows max 180A
  >= 25°C without limit (200A)
  >= 15°C BMS allows max 120A
  >= 5°C BMS allows max 90A
  >= 1°C BMS allows max 60A
  <= 0°C BMS allows max 40A
*/

#include <Arduino.h>
#include <stdint.h>
#include <WString.h>
#include <string.h>
#include <sys/time.h>
#include "LiveData.h"
#include "CarGeely.h"
#include "CommInterface.h"
#include <vector>

#define commandQueueLoopFromKiaENiro 8

/**
   activateCommandQueue
*/
void CarGeely::activateCommandQueue()
{

  // Optimizer
  lastAllowTpms = 0;

  // Command queue
  //https://www.drive2.ru/b/499095541834055933/?ysclid=lqwjcmy2th697301562
  //https://habr.com/ru/articles/544144/
  //https://canhacker.ru/protocol-uds/
  std::vector<String> commandQueueKiaENiro = {
      "ATZ",    // Reset all
      "ATI",    // Print the version ID
      "ATS0",   // Printing of spaces on
      "ATE0",   // Echo off
      "ATL0",   // Linefeeds off
      "ATSP6", // Select protocol to ISO 15765-4 CAN (11 bit ID, 500 kbit/s)
      //"ATAL",     // Allow Long (>7 byte) messages
      //"ATAR",     // Automatically receive
      //"AT H1",     // Headers on (debug only)
      //"AT D1",     // Display of the DLC on
      //"AT CAF0",   // Automatic formatting off
      ////"AT AT0",     // disabled adaptive timing
      "ATDP",
      "ATST16", // reduced timeout to 1, orig.16

      // Standard OBD2 PIDs
      "ATSH7DF",
      "0105", // Coolant temp
      "012F", // Fuel level %
      "013C", // Catalist temp

      // Vendor Engine ECU
      "ATSH7E0",
      "220156", // Engine oil temp
      "220105", // Engine oil level mm

      // Vendor AT ECU
      "ATSH7E1",
      "220A12", // AT oil temp
  };

  // 39 or 64 kWh model?
  liveData->params.batModuleTempCount = 4;
  liveData->params.batteryTotalAvailableKWh = 64;
  liveData->params.cellCount = 98;

  //  Empty and fill command queue
  liveData->commandQueue.clear();
  for (auto cmd : commandQueueKiaENiro)
  {
    liveData->commandQueue.push_back({0, cmd}); // stxChar not used, keep it 0
  }

  liveData->commandQueueLoopFrom = commandQueueLoopFromKiaENiro;
  liveData->commandQueueCount = commandQueueKiaENiro.size();
  liveData->rxTimeoutMs = 500;            // timeout for receiving of CAN response
  liveData->delayBetweenCommandsMs = 100; // delay between commands, set to 0 if no delay is needed
}

/**
   parseRowMerged
*/
void CarGeely::parseRowMerged()
{

  uint8_t tempByte;
  //  float tempFloat;
  String tmpStr;
  
  if (liveData->responseID == 0x7E8)
  {
    if (liveData->responseRowMerged.startsWith("4105"))
    {
      //41 05 63 AA AA AA AA
      liveData->params.coolantTemp1C = liveData->hexToDecFromResponse(4, 6, 1, false) - 40; 
    }
    else if (liveData->responseRowMerged.startsWith("413C"))
    {
      //41 3C 12 12 AA AA AA
      auto par1 = liveData->hexToDecFromResponse(4, 6, 1, false);
      auto par2 = liveData->hexToDecFromResponse(6, 8, 1, false);
      liveData->params.catalystTempC = (256 * par1 + par2) / 10 - 40;
    }
  }
  else if (liveData->responseID == 0x7E9)
  {
    if (liveData->responseRowMerged.startsWith("620A12"))
    {
      //62 0A 12 57 AA AA AA
      liveData->params.coolantTemp2C = liveData->hexToDecFromResponse(6, 8, 1, false) - 40;
    }
  }
}

/**
   Is command from queue allowed for execute, or continue with next
*/
bool CarGeely::commandAllowed()
{

  /* syslog->print("Command allowed: ");
    syslog->print(liveData->currentAtshRequest);
    syslog->print(" ");
    syslog->println(liveData->commandRequest);*/

  // SleepMode Queue Filter
  if (liveData->params.sleepModeQueue)
  {
    if (liveData->commandQueueIndex < liveData->commandQueueLoopFrom)
    {
      return true;
    }
    if (liveData->commandRequest.equals("ATSH7E4"))
    {
      return true;
    }
    if (liveData->currentAtshRequest.equals("ATSH7E4") && liveData->commandRequest.equals("220105"))
    {
      return true;
    }

    return false;
  }

  // Disabled command optimizer (allows to log all car values to sdcard, but it's slow)
  if (liveData->settings.disableCommandOptimizer) {
    return true;
  }

  // TPMS (once per 30 secs.)
  if (liveData->commandRequest.equals("ATSH7A0"))
  {
    return lastAllowTpms + 30 < liveData->params.currentTime;
  }
  if (liveData->currentAtshRequest.equals("ATSH7A0") && liveData->commandRequest.equals("22C00B"))
  {
    if (lastAllowTpms + 30 < liveData->params.currentTime)
    {
      lastAllowTpms = liveData->params.currentTime;
    }
    else
    {
      return false;
    }
  }

  // BMS (only for SCREEN_CELLS)
  if (liveData->currentAtshRequest.equals("ATSH7E4"))
  {
    if (liveData->commandRequest.equals("220102") || liveData->commandRequest.equals("220103") || liveData->commandRequest.equals("220104"))
    {
      if (liveData->params.displayScreen != SCREEN_CELLS && liveData->params.displayScreenAutoMode != SCREEN_CELLS && liveData->settings.sdcardEnabled != 1)
        return false;
    }
  }

  // HUD speedup
  if (liveData->params.displayScreen == SCREEN_HUD)
  {
    // no cooling water temp
    if (liveData->currentAtshRequest.equals("ATSH7E4"))
    {
      if (liveData->commandRequest.equals("220106"))
      {
        return false;
      }
    }

    // no aircondition
    if (liveData->currentAtshRequest.equals("ATSH7B3"))
    {
      return false;
    }

    // no ODO
    if (liveData->currentAtshRequest.equals("ATSH7C6"))
    {
      return false;
    }

    // no BCM / TPMS
    if (liveData->currentAtshRequest.equals("ATSH7A0"))
    {
      return false;
    }

    // no AUX
    if (liveData->currentAtshRequest.equals("ATSH7E2") && liveData->commandRequest.equals("2102"))
    {
      return false;
    }
  }

  return true;
}

/**
   loadTestData
*/
void CarGeely::loadTestData()
{

  // IGPM
  liveData->currentAtshRequest = "ATSH770";
  // 22BC03
  liveData->commandRequest = "22BC03";
  liveData->responseRowMerged = "62BC03FDEE7C730A600000AAAA";
  parseRowMerged();

  // ABS / ESP + AHB ATSH7D1
  liveData->currentAtshRequest = "ATSH7D1";
  // 2101
  liveData->commandRequest = "22C101";
  liveData->responseRowMerged = "62C1015FD7E7D0FFFF00FF04D0D400000000FF7EFF0030F5010000FFFF7F6307F207FE05FF00FF3FFFFFAAAAAAAAAAAA";
  parseRowMerged();

  // VMCU ATSH7E2
  liveData->currentAtshRequest = "ATSH7E2";
  // 2101
  liveData->commandRequest = "2101";
  liveData->responseRowMerged = "6101FFF8000009285A3B0648030000B4179D763404080805000000";
  parseRowMerged();
  // 2102
  liveData->commandRequest = "2102";
  liveData->responseRowMerged = "6102F8FFFC000101000000840FBF83BD33270680953033757F59291C76000001010100000007000000";
  liveData->responseRowMerged = "6102F8FFFC000101000000931CC77F4C39040BE09BA7385D8158832175000001010100000007000000";
  parseRowMerged();

  // "ATSH7DF",
  liveData->currentAtshRequest = "ATSH7DF";
  // 2106
  liveData->commandRequest = "2106";
  liveData->responseRowMerged = "6106FFFF800000000000000200001B001C001C000600060006000E000000010000000000000000013D013D013E013E00";
  parseRowMerged();

  // AIRCON / ACU ATSH7B3
  liveData->currentAtshRequest = "ATSH7B3";
  // 220100
  liveData->commandRequest = "220100";
  liveData->responseRowMerged = "6201007E5027C8FF7F765D05B95AFFFF5AFF11FFFFFFFFFFFF6AFFFF2DF0757630FFFF00FFFF000000";
  liveData->responseRowMerged = "6201007E5027C8FF867C58121010FFFF10FF8EFFFFFFFFFFFF10FFFF0DF0617900FFFF01FFFF000000";
  parseRowMerged();

  // BMS ATSH7E4
  liveData->currentAtshRequest = "ATSH7E4";
  // 220101
  liveData->commandRequest = "220101";
  liveData->responseRowMerged = "620101FFF7E7FF99000000000300B10EFE120F11100F12000018C438C30B00008400003864000035850000153A00001374000647010D017F0BDA0BDA03E8";
  liveData->responseRowMerged = "620101FFF7E7FFB3000000000300120F9B111011101011000014CC38CB3B00009100003A510000367C000015FB000013D3000690250D018E0000000003E8";
  parseRowMerged();
  // 220102
  liveData->commandRequest = "220102";
  liveData->responseRowMerged = "620102FFFFFFFFCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBAAAA";
  parseRowMerged();
  // 220103
  liveData->commandRequest = "220103";
  liveData->responseRowMerged = "620103FFFFFFFFCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCACBCACACFCCCBCBCBCBCBCBCBCBAAAA";
  parseRowMerged();
  // 220104
  liveData->commandRequest = "220104";
  liveData->responseRowMerged = "620104FFFFFFFFCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBCBAAAA";
  parseRowMerged();
  // 220105
  liveData->commandRequest = "220105";
  liveData->responseRowMerged = "620105003fff9000000000000000000F8A86012B4946500101500DAC03E800000000AC0000C7C701000F00000000AAAA";
  liveData->responseRowMerged = "620105003FFF90000000000000000014918E012927465000015013BB03E800000000BB0000CBCB01001300000000AAAA";
  parseRowMerged();
  // 220106
  liveData->commandRequest = "220106";
  liveData->responseRowMerged = "620106FFFFFFFF14001A00240000003A7C86B4B30000000928EA00";
  parseRowMerged();

  // BCM / TPMS ATSH7A0
  liveData->currentAtshRequest = "ATSH7A0";
  // 22C00B
  liveData->commandRequest = "22C00B";
  liveData->responseRowMerged = "62C00BFFFF0000B93D0100B43E0100B43D0100BB3C0100AAAAAAAA";
  parseRowMerged();

  // ATSH7C6
  liveData->currentAtshRequest = "ATSH7C6";
  // 22b002
  liveData->commandRequest = "22B002";
  liveData->responseRowMerged = "62B002E0000000FFB400330B0000000000000000";
  parseRowMerged();

  liveData->params.batModuleTempC[0] = 28;
  liveData->params.batModuleTempC[1] = 29;
  liveData->params.batModuleTempC[2] = 28;
  liveData->params.batModuleTempC[3] = 30;

  // This is more accurate than min/max from BMS. It's required to detect kona/eniro cold gates (min 15C is needed > 43kW charging, min 25C is needed > 58kW charging)
  liveData->params.batMinC = liveData->params.batMaxC = liveData->params.batModuleTempC[0];
  for (uint16_t i = 1; i < liveData->params.batModuleTempCount; i++)
  {
    if (liveData->params.batModuleTempC[i] < liveData->params.batMinC)
      liveData->params.batMinC = liveData->params.batModuleTempC[i];
    if (liveData->params.batModuleTempC[i] > liveData->params.batMaxC)
      liveData->params.batMaxC = liveData->params.batModuleTempC[i];
  }
  liveData->params.batTempC = liveData->params.batMinC;

  //
  liveData->params.soc10ced[10] = 2200;
  liveData->params.soc10cec[10] = 2500;
  liveData->params.soc10odo[10] = 13000;
  liveData->params.soc10time[10] = 13000;
  liveData->params.soc10ced[9] = liveData->params.soc10ced[10] + 6.4;
  liveData->params.soc10cec[9] = liveData->params.soc10cec[10] + 0;
  liveData->params.soc10odo[9] = liveData->params.soc10odo[10] + 30;
  liveData->params.soc10time[9] = liveData->params.soc10time[10] + 900;
  liveData->params.soc10ced[8] = liveData->params.soc10ced[9] + 6.8;
  liveData->params.soc10cec[8] = liveData->params.soc10cec[9] + 0;
  liveData->params.soc10odo[8] = liveData->params.soc10odo[9] + 30;
  liveData->params.soc10time[8] = liveData->params.soc10time[9] + 900;
  liveData->params.soc10ced[7] = liveData->params.soc10ced[8] + 7.2;
  liveData->params.soc10cec[7] = liveData->params.soc10cec[8] + 0.6;
  liveData->params.soc10odo[7] = liveData->params.soc10odo[8] + 30;
  liveData->params.soc10time[7] = liveData->params.soc10time[8] + 900;
  liveData->params.soc10ced[6] = liveData->params.soc10ced[7] + 6.7;
  liveData->params.soc10cec[6] = liveData->params.soc10cec[7] + 0;
  liveData->params.soc10odo[6] = liveData->params.soc10odo[7] + 30;
  liveData->params.soc10time[6] = liveData->params.soc10time[7] + 900;
  liveData->params.soc10ced[5] = liveData->params.soc10ced[6] + 6.7;
  liveData->params.soc10cec[5] = liveData->params.soc10cec[6] + 0;
  liveData->params.soc10odo[5] = liveData->params.soc10odo[6] + 30;
  liveData->params.soc10time[5] = liveData->params.soc10time[6] + 900;
  liveData->params.soc10ced[4] = liveData->params.soc10ced[5] + 6.4;
  liveData->params.soc10cec[4] = liveData->params.soc10cec[5] + 0.3;
  liveData->params.soc10odo[4] = liveData->params.soc10odo[5] + 30;
  liveData->params.soc10time[4] = liveData->params.soc10time[5] + 900;
  liveData->params.soc10ced[3] = liveData->params.soc10ced[4] + 6.4;
  liveData->params.soc10cec[3] = liveData->params.soc10cec[4] + 0;
  liveData->params.soc10odo[3] = liveData->params.soc10odo[4] + 30;
  liveData->params.soc10time[3] = liveData->params.soc10time[4] + 900;
  liveData->params.soc10ced[2] = liveData->params.soc10ced[3] + 5.4;
  liveData->params.soc10cec[2] = liveData->params.soc10cec[3] + 0.1;
  liveData->params.soc10odo[2] = liveData->params.soc10odo[3] + 30;
  liveData->params.soc10time[2] = liveData->params.soc10time[3] + 900;
  liveData->params.soc10ced[1] = liveData->params.soc10ced[2] + 6.2;
  liveData->params.soc10cec[1] = liveData->params.soc10cec[2] + 0.1;
  liveData->params.soc10odo[1] = liveData->params.soc10odo[2] + 30;
  liveData->params.soc10time[1] = liveData->params.soc10time[2] + 900;
  liveData->params.soc10ced[0] = liveData->params.soc10ced[1] + 2.9;
  liveData->params.soc10cec[0] = liveData->params.soc10cec[1] + 0.5;
  liveData->params.soc10odo[0] = liveData->params.soc10odo[1] + 15;
  liveData->params.soc10time[0] = liveData->params.soc10time[1] + 900;

  // DEMO DATA
  liveData->params.brakeLights = true;
  liveData->params.headLights = true;
  liveData->settings.sdcardEnabled = 1;
  liveData->params.queueLoopCounter = 1;
  liveData->params.motorTempC = 4;
  liveData->params.inverterTempC = 3;
  liveData->params.trunkDoorOpen = true;
  liveData->params.leftFrontDoorOpen = true;
  liveData->params.rightFrontDoorOpen = true;
  liveData->params.leftRearDoorOpen = true;
  liveData->params.rightRearDoorOpen = true;
  liveData->params.hoodDoorOpen = true;
  liveData->params.auxVoltage = 12.7;
}

/**
   Test handler
*/
void CarGeely::testHandler(const String &cmd)
{
  int8_t idx = cmd.indexOf("/");
  if (idx == -1)
    return;
  String key = cmd.substring(0, idx);
  String value = cmd.substring(idx + 1);

  // AIRCON SCANNER
  if (key.equals("bms"))
  {
    syslog->println("Scanning...");

    // SET TESTER PRESENT
    commInterface->sendPID(liveData->hexToDec("0770", 2, false), "3E");
    delay(10);
    for (uint16_t i = 0; i < (liveData->rxTimeoutMs / 20); i++)
    {
      if (commInterface->receivePID() != 0xff)
        break;
      delay(20);
    }
    delay(liveData->delayBetweenCommandsMs);

    // CHANGE SESSION
    commInterface->sendPID(liveData->hexToDec("0770", 2, false), "1003");
    delay(10);
    for (uint16_t i = 0; i < (liveData->rxTimeoutMs / 20); i++)
    {
      if (commInterface->receivePID() != 0xff)
      {
        // WAIT FOR POSITIVE ANSWER
        if (liveData->responseRowMerged.equals("5003"))
        {
          syslog->println("POSITIVE ANSWER");
          break;
        }
      }
      delay(20);
    }
    delay(liveData->delayBetweenCommandsMs);

    // test=bms/1
    for (uint16_t a = 188; a < 255; a++)
    {
      syslog->print("NEW CYCLE: ");
      syslog->println(a);
      for (uint16_t b = 0; b < 255; b++)
      // for (uint16_t c = 0; c < 255; c++)
      {
        String command = "2F";
        if (a < 16)
          command += "0";
        command += String(a, HEX);
        if (b < 16)
          command += "0";
        command += String(b, HEX);
        /*if (c < 16)
            command += "0";
          command += String(c, HEX);
        */
        command.toUpperCase();
        command += "00";

        // EXECUTE COMMAND
        // syslog->print(".");
        commInterface->sendPID(liveData->hexToDec("0770", 2, false), command);
        //      syslog->setDebugLevel(DEBUG_COMM);
        delay(10);
        for (uint16_t i = 0; i < (liveData->rxTimeoutMs / 20); i++)
        {
          if (commInterface->receivePID() != 0xff)
          {
            if (!liveData->prevResponseRowMerged.equals("7F2F31") /*&& !liveData->prevResponseRowMerged.equals("")*/)
            {
              syslog->print("### \t");
              syslog->print(command);
              syslog->print(" \t");
              syslog->println(liveData->prevResponseRowMerged);
            }
            break;
          }
          delay(10);
        }
        delay(liveData->delayBetweenCommandsMs);
        //      syslog->setDebugLevel(liveData->settings.debugLevel);
      }
    }
  }
  // ECU SCAN
  else if (key.equals("ecu"))
  {
    // test=ecu/1
    for (uint16_t unit = 1904; unit < 2047; unit++)
    {
      String command = "2101"; /*
       if (i < 16)
         command += "0";
       command += String(i, HEX);
       command.toUpperCase();
       command += "01";*/

      eNiroCarControl(unit, command);
      // WAIT FOR POSITIVE ANSWER
      if (liveData->responseRowMerged.equals("7F2111"))
      {
        syslog->print(unit);
        syslog->println(" POSITIVE ANSWER");
      }
    }
  }
  // BATCH SCAN
  else if (key.equals("batch"))
  {
    // test=batch/1
    for (uint16_t i = 0; i < 250; i++)
    {
      String command = "2F";
      if (i < 16)
        command += "0";
      command += String(i, HEX);
      command.toUpperCase();
      command += "0100";

      syslog->print(command);
      syslog->print(" ");

      eNiroCarControl(liveData->hexToDec("07B3", 2, false), command);
    }
  }
  // ONE COMMAND
  else
  {
    // test=07C6/2FB00103
    eNiroCarControl(liveData->hexToDec(key, 2, false), value);
  }
}

/**
 * Custom menu
 */
std::vector<String> CarGeely::customMenu(int16_t menuId)
{
  if (menuId == MENU_CAR_COMMANDS)
    return {
        "vessOn=VESS 5sec.",
        "doorsUnlock=Unlock doors",
        "doorsLock=Lock doors",
        "chargeCableLockOff=Charge cable lock off",
        "chargeCableLockOn=Charge cable lock on",
        "roomLampOff=Room lamp off",
        "roomLampOn=Room lamp on",
        "luggageLampOff=Luggage lamp off",
        "luggageLampOn=Luggage lamp on",
        "mirrorsUnfold=Unfold mirrors",
        "mirrorsFold=Fold mirrors",
        "heatSteeringWheelOff=Heat steering wheel off",
        "heatSteeringWheelOn=Heat steering wheel on",
        "clusterIndicatorsOff=Cluster indicators off",
        "clusterIndicatorsOn=Cluster indicators on",
        "turnSignalLeftOff=Turn signal left off",
        "turnSignalLeftOn=Turn signal left on",
        "turnSignalRightOff=Turn signal right off",
        "turnSignalRightOn=Turn signal right on",
        "headLightLowOff=Head light low off",
        "headLightLowOn=Head light low on",
        "headLightHighOff=Head light high off",
        "headLightHighOn=Head light high on",
        "frontFogLightOff=Front fog light off",
        "frontFogLightOn=Front fog light on",
        "rearLightOff=Rear light off",
        "rearLightOn=Rear light on",
        "rearFogLightOff=Rear fog light off",
        "rearFogLightOn=Rear fog light on",
        "rearDefoggerOff=Rear deffoger off",
        "rearDefoggerOn=Rear deffoger on",
        "rearLeftBrakeLightOff=Left brake light off",
        "rearLeftBrakeLightOn=Left brake light on",
        "rearRightBrakeLightOff=Right brake light off",
        "rearRightBrakeLightOn=Right brake light on",
    };

  return {};
}

/**
 * Execute custom command
 */
void CarGeely::carCommand(const String &cmd)
{
  if (cmd.equals("vessOn"))
  {
    eNiroCarControl(0x736, "2FF01103");
  }
  if (cmd.equals("doorsUnlock"))
  {
    eNiroCarControl(0x770, "2FBC1103");
  }
  if (cmd.equals("doorsLock"))
  {
    eNiroCarControl(0x770, "2FBC1003");
  }
  if (cmd.equals("chargeCableLockOff"))
  {
    eNiroCarControl(0x770, "2FBC4103");
  }
  if (cmd.equals("chargeCableLockOn"))
  {
    eNiroCarControl(0x770, "2FBC3F03");
  }
  if (cmd.equals("roomLampOff"))
  {
    eNiroCarControl(0x7A0, "2FB01900");
  }
  if (cmd.equals("roomLampOn"))
  {
    eNiroCarControl(0x7A0, "2FB01903");
  }
  if (cmd.equals("luggageLampOff"))
  {
    eNiroCarControl(0x770, "2FBC1C00");
  }
  if (cmd.equals("luggageLampOn"))
  {
    eNiroCarControl(0x770, "2FBC1C03");
  }
  if (cmd.equals("mirrorsUnfold"))
  {
    eNiroCarControl(0x7A0, "2FB05C03");
  }
  if (cmd.equals("mirrorsFold"))
  {
    eNiroCarControl(0x7A0, "2FB05B03");
  }
  if (cmd.equals("heatSteeringWheelOff"))
  {
    eNiroCarControl(0x7A0, "2FB05900"); // heat power
    eNiroCarControl(0x7A0, "2FB05A00"); // LED indicator
  }
  if (cmd.equals("heatSteeringWheelOn"))
  {
    eNiroCarControl(0x7A0, "2FB05903"); // heat power
    eNiroCarControl(0x7A0, "2FB05A03"); // LED indicator
  }
  if (cmd.equals("clusterIndicatorsOff"))
  {
    eNiroCarControl(0x7C6, "2FB00100");
  }
  if (cmd.equals("clusterIndicatorsOn"))
  {
    eNiroCarControl(0x7C6, "2FB00103");
  }
  if (cmd.equals("turnSignalLeftOff"))
  {
    eNiroCarControl(0x770, "2FBC1500");
  }
  if (cmd.equals("turnSignalLeftOn"))
  {
    eNiroCarControl(0x770, "2FBC1503");
  }
  if (cmd.equals("turnSignalRightOff"))
  {
    eNiroCarControl(0x770, "2FBC1600");
  }
  if (cmd.equals("turnSignalRightOn"))
  {
    eNiroCarControl(0x770, "2FBC1603");
  }
  if (cmd.equals("headLightLowOff"))
  {
    eNiroCarControl(0x770, "2FBC0100");
  }
  if (cmd.equals("headLightLowOn"))
  {
    eNiroCarControl(0x770, "2FBC0103");
  }
  if (cmd.equals("headLightHighOff"))
  {
    eNiroCarControl(0x770, "2FBC0200");
  }
  if (cmd.equals("headLightHighOn"))
  {
    eNiroCarControl(0x770, "2FBC0203");
  }
  if (cmd.equals("frontFogLightOff"))
  {
    eNiroCarControl(0x770, "2FBC0300");
  }
  if (cmd.equals("frontFogLightOn"))
  {
    eNiroCarControl(0x770, "2FBC0303");
  }
  if (cmd.equals("rearLightOff"))
  {
    eNiroCarControl(0x770, "2FBC0400");
  }
  if (cmd.equals("rearLightOn"))
  {
    eNiroCarControl(0x770, "2FBC0403");
  }
  if (cmd.equals("rearFogLightOff"))
  {
    eNiroCarControl(0x770, "2FBC0800");
  }
  if (cmd.equals("rearFogLightOn"))
  {
    eNiroCarControl(0x770, "2FBC0803");
  }
  if (cmd.equals("rearDefoggerOff"))
  {
    eNiroCarControl(0x770, "2FBC0C00");
  }
  if (cmd.equals("rearDefoggerOn"))
  {
    eNiroCarControl(0x770, "2FBC0C03");
  }
  if (cmd.equals("rearLeftBrakeLightOff"))
  {
    eNiroCarControl(0x770, "2FBC2B00");
  }
  if (cmd.equals("rearLeftBrakeLightOn"))
  {
    eNiroCarControl(0x770, "2FBC2B03");
  }
  if (cmd.equals("rearRightBrakeLightOff"))
  {
    eNiroCarControl(0x770, "2FBC2C00");
  }
  if (cmd.equals("rearRightBrakeLightOn"))
  {
    eNiroCarControl(0x770, "2FBC2C03");
  }
}

/**
 * Eniro cmds
 */
void CarGeely::eNiroCarControl(const uint16_t pid, const String &cmd)
{
  // syslog->println("EXECUTING COMMAND");
  // syslog->println(cmd);
  commInterface->sendPID(pid, "3E"); // SET TESTER PRESENT
  delay(10);
  for (uint16_t i = 0; i < (liveData->rxTimeoutMs / 20); i++)
  {
    if (commInterface->receivePID() != 0xff)
      break;
    delay(20);
  }
  delay(liveData->delayBetweenCommandsMs);

  commInterface->sendPID(pid, "1003"); // CHANGE SESSION
  delay(10);
  for (uint16_t i = 0; i < (liveData->rxTimeoutMs / 20); i++)
  {
    if (commInterface->receivePID() != 0xff)
    {
      // WAIT FOR POSITIVE ANSWER
      if (liveData->responseRowMerged.equals("5003"))
      {
        break;
      }
    }
    delay(20);
  }
  delay(liveData->delayBetweenCommandsMs);

  // EXECUTE COMMAND
  commInterface->sendPID(pid, cmd);
  syslog->setDebugLevel(DEBUG_COMM);
  delay(10);
  for (uint16_t i = 0; i < (liveData->rxTimeoutMs / 20); i++)
  {
    if (commInterface->receivePID() != 0xff)
      break;
    delay(20);
  }
  delay(liveData->delayBetweenCommandsMs);

  syslog->setDebugLevel(liveData->settings.debugLevel);
}
