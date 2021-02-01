#pragma once

#include "config.h"

MENU_ITEM menuItemsSource[100] = {

    {0, 0, 0, "<- exit menu"},
    {MENU_VEHICLE_TYPE, 0, -1, "Vehicle type"},
    {MENU_ADAPTER_TYPE, 0, -1, "Adapter type"},
    {2, 0, -1, "Select OBD2 BLE4 adapter"},
    {3, 0, -1, "Others"},
    {4, 0, -1, "Units"},
    {8, 0, -1, "Factory reset"},
    {MENU_SAVE_SETTINGS, 0, -1, "Save settings"},
    {MENU_APP_VERSION, 0, -1, "Version"},
    {MENU_SHUTDOWN, 0, -1, "Shutdown"},
    {MENU_CAR_COMMANDS, 0, -1, "Car commands [CAN]"},

    {100, 1, 0, "<- parent menu"},
    {101, 1, -1, "Kia eNiro 2020 64kWh"},
    {104, 1, -1, "Kia eNiro 2020 39kWh"},
    {109, 1, -1, "Kia eSoul 2020 64kWh"},
    {103, 1, -1, "Hyundai Ioniq 2018 28kWh"},
    {102, 1, -1, "Hyundai Kona 2020 64kWh"},
    {105, 1, -1, "Hyundai Kona 2020 39kWh"},
    {106, 1, -1, "Renault Zoe 22kWh (DEV)"},
    {107, 1, -1, "Kia Niro PHEV 8.9kWh (DEV)"},
    {108, 1, -1, "BMW i3 2014 22kWh (DEV)"},
    {120, 1, -1, "Debug OBD2 Kia"},

    {MENU_ADAPTER_BLE4 - 1, MENU_ADAPTER_TYPE, 0, "<- parent menu"},
    {MENU_ADAPTER_BLE4, MENU_ADAPTER_TYPE, -1, "Bluetooth 4 (BLE4)"},
    {MENU_ADAPTER_CAN, MENU_ADAPTER_TYPE, -1, "CAN bus (MCP2515-1/SO)"},
    //{MENU_ADAPTER_BT3, MENU_ADAPTER_TYPE, -1,  "Bluetooth 3 (dev)"},

    {300, 3, 0, "<- parent menu"},
    // {MENU_WIFI, 3, -1, "[dev] WiFi network"},
    {MENU_SDCARD, 3, -1, "SD card"},
    {MENU_GPS, 3, -1, "GPS"},
    {MENU_REMOTE_UPLOAD, 3, -1, "Rem. Upload"},
    {MENU_SERIAL_CONSOLE, 3, -1, "Serial console"},
    {MENU_VOLTMETER, 3, -1, "Voltmeter INA3221"},
    {MENU_DEBUG_LEVEL, 3, -1, "Debug level"},
    //{MENU_NTP, 3, -1, "[dev] NTP"},
    {MENU_SCREEN_ROTATION, 3, -1, "Screen rotation"},
    {MENU_DEFAULT_SCREEN, 3, -1, "Default screen"},
    {MENU_SCREEN_BRIGHTNESS, 3, -1, "LCD brightness"},
    {MENU_PREDRAWN_GRAPHS, 3, -1, "Pre-drawn ch.graphs"},
    {MENU_HEADLIGHTS_REMINDER, 3, -1, "Headlight reminder"},
    {MENU_SLEEP_MODE, 3, -1, "SleepMode"},

    {400, 4, 0, "<- parent menu"},
    {MENU_DISTANCE_UNIT, 4, -1, "Distance"},
    {MENU_TEMPERATURE_UNIT, 4, -1, "Temperature"},
    {MENU_PRESSURE_UNIT, 4, -1, "Pressure"},

    {3010, 301, 3, "<- parent menu"},
    {MENU_WIFI_ENABLED, 301, -1, "WiFi enabled"},
    {MENU_WIFI_SSID, 301, -1, "SSID"},
    {MENU_WIFI_PASSWORD, 301, -1, "Password"},

    {MENU_SDCARD * 10, MENU_SDCARD, 3, "<- parent menu"},
    {MENU_SDCARD_ENABLED, MENU_SDCARD, -1, "SD enabled"},
    {MENU_SDCARD_AUTOSTARTLOG, MENU_SDCARD, -1, "Autostart log enabled"},
    {MENU_SDCARD_MOUNT_STATUS, MENU_SDCARD, -1, "Status"},
    {MENU_SDCARD_REC, MENU_SDCARD, -1, "Record"},
    //{MENU_SDCARD_INTERVAL, MENU_SDCARD, -1, "Log interval sec."},

    {MENU_VOLTMETER * 10, MENU_VOLTMETER, 3, "<- parent menu"},
    {MENU_VOLTMETER_ENABLED, MENU_VOLTMETER, -1, "Voltmeter enabled"},
    {MENU_VOLTMETER_SLEEP, MENU_VOLTMETER, -1, "Control SleepMode"},
    {MENU_VOLTMETER_SLEEPVOL, MENU_VOLTMETER, -1, "Sleep Vol."},
    {MENU_VOLTMETER_WAKEUPVOL, MENU_VOLTMETER, -1, "WakeUp Vol."},
    {MENU_VOLTMETER_CUTOFFVOL, MENU_VOLTMETER, -1, "CutOff Vol."},

    {MENU_SLEEP_MODE * 10, MENU_SLEEP_MODE, 3, "<- parent menu"},
    {MENU_SLEEP_MODE_MODE, MENU_SLEEP_MODE, -1, "Mode"},
    {MENU_SLEEP_MODE_WAKEINTERVAL, MENU_SLEEP_MODE, -1, "WakeUp Check"},
    {MENU_SLEEP_MODE_SHUTDOWNHRS, MENU_SLEEP_MODE, -1, "Shutdown After"},

    {MENU_REMOTE_UPLOAD * 10, MENU_REMOTE_UPLOAD, 3, "<- parent menu"},
    {MENU_REMOTE_UPLOAD_UART, MENU_REMOTE_UPLOAD, -1, "SerialConsole"},
    {MENU_REMOTE_UPLOAD_TYPE, MENU_REMOTE_UPLOAD, -1, "Module Type"},
    {MENU_REMOTE_UPLOAD_API_INTERVAL, MENU_REMOTE_UPLOAD, -1, "API Upload Int."},
    {MENU_REMOTE_UPLOAD_ABRP_INTERVAL, MENU_REMOTE_UPLOAD, -1, "ABRP Upload Int."},

    {3060, 306, 3, "<- parent menu"},
    {3061, 306, -1, "Auto mode"},
    {3062, 306, -1, "Basic info"},
    {3063, 306, -1, "Speed"},
    {3064, 306, -1, "Battery cells"},
    {3065, 306, -1, "Charging graph"},
    {3066, 306, -1, "HUD"},

    {4010, 401, 4, "<- parent menu"},
    {4011, 401, -1, "Kilometers"},
    {4012, 401, -1, "Miles"},

    {4020, 402, 4, "<- parent menu"},
    {4021, 402, -1, "Celsius"},
    {4022, 402, -1, "Fahrenheit"},

    {4030, 403, 4, "<- parent menu"},
    {4031, 403, -1, "Bar"},
    {4032, 403, -1, "Psi"},

    {9999, 9998, 0, "List of BLE devices"},
    {10000, 9999, 0, "<- parent menu"},
    {10001, 9999, -1, "-"},
    {10002, 9999, -1, "-"},
    {10003, 9999, -1, "-"},
    {10004, 9999, -1, "-"},
    {10005, 9999, -1, "-"},
    {10006, 9999, -1, "-"},
    {10007, 9999, -1, "-"},
    {10008, 9999, -1, "-"},
    {10009, 9999, -1, "-"},

    {MENU_CAR_COMMANDS * 1000, MENU_CAR_COMMANDS, 0, "<- parent menu"},
};