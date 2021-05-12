#include <Arduino.h>
#include <ELMduino.h>
#include <BluetoothSerial.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/touch_pad.h"
#include "driver/periph_ctrl.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "nvs_flash.h"
#include <LCDWIKI_GUI.h>
#include <SSD1283A.h>

#define BLACK 0x0000
#define CYAN 0x07FF
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define ORANGE 0xF900
#define RED 0xF800
#define MAGENTA 0xF81F

#define CORE_0 1 // Core 0 does not work?
#define CORE_1 1

#define PAIR_MAX_DEVICES 3

#define TOUCH_PAD_NO_CHANGE (-1)
#define TOUCH_THRESH_NO_USE (0)
#define TOUCH_FILTER_MODE_EN (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

#define BOOST_RESET_VALUE 99
#define TEMP_RESET_VALUE -39

#define DEBUG
//#define DEBUG_WATERMARK

#ifdef DEBUG
#define DEBUG_PRINTS(x) printf(x);
#define DEBUG_PRINTSS(x, y) printf(x, y);
#else
#define DEBUG_PRINTS(x) ;
#define DEBUG_PRINTSS(x, y) ;
#endif

BluetoothSerial SerialBT;
ELM327 myELM327;
SSD1283A_GUI tft(/*CS*/ 5, /*CD*/ 33, /*RST*/ 32, /*LED*/ 25);

static QueueHandle_t xQueueBoost;
static QueueHandle_t xQueueIAT;
static QueueHandle_t xQueueOil;
static QueueHandle_t xQueueCoolant;
static QueueHandle_t xQueueTimingAdvance;
static QueueHandle_t xQueueHPFPPressure;

static QueueHandle_t xQueueBoostMaxValue;
static QueueHandle_t xQueueIATMaxValue;
static QueueHandle_t xQueueOilMaxValue;
static QueueHandle_t xQueueCoolantMaxValue;

static SemaphoreHandle_t xSemaphore;

void vWaitForOK(void)
{
    char cPayload[64];

    for (register uint8_t i = 0; (i <= sizeof(cPayload) - 1); i++)
        cPayload[i] = '\0';

    if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
    {
        if (SerialBT.available())
            SerialBT.readBytes(cPayload, sizeof(cPayload) - 1);

        if (strstr(cPayload, "OK\r\r>") != NULL)
        {
            DEBUG_PRINTS("\nOK");
            return;
        }
        else
        {
            DEBUG_PRINTS("\nNOT OK");
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        xSemaphoreGive(xSemaphore);
    }
}

UBaseType_t uxCheckHighWaterMark(void)
{
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    return uxHighWaterMark;
}

BaseType_t isPressed(void)
{
    uint16_t ui16TouchValueFiltered = 0;

    touch_pad_read_filtered((touch_pad_t)0, &ui16TouchValueFiltered);
    BaseType_t bReturn = ui16TouchValueFiltered < 600 ? pdTRUE : pdFALSE;

    return bReturn;
}

void vBoostColor(uint8_t ucBoost)
{
    tft.Set_Text_Back_colour(BLACK);
    tft.Set_Text_Size(5);

    if (ucBoost <= 229)
        tft.Set_Text_colour(WHITE);
    else if ((ucBoost >= 230) && (ucBoost <= 239))
        tft.Set_Text_colour(YELLOW);
    else if ((ucBoost >= 240) && (ucBoost <= 249))
        tft.Set_Text_colour(ORANGE);
    else if (ucBoost >= 250)
        tft.Set_Text_colour(RED);
}

void vIATColor(int8_t cIAT)
{
    tft.Set_Text_Back_colour(BLACK);
    tft.Set_Text_Size(3);

    if (cIAT <= 39)
        tft.Set_Text_colour(WHITE);
    else if ((cIAT >= 40) && (cIAT <= 49))
        tft.Set_Text_colour(YELLOW);
    else if ((cIAT >= 50) && (cIAT <= 59))
        tft.Set_Text_colour(ORANGE);
    else if (cIAT >= 60)
        tft.Set_Text_colour(RED);
}

void vOilTempColor(int8_t cOilTemperature)
{
    tft.Set_Text_Back_colour(BLACK);
    tft.Set_Text_Size(3);

    if (cOilTemperature <= 69)
        tft.Set_Text_colour(CYAN);
    else if ((cOilTemperature >= 70) && (cOilTemperature <= 89))
        tft.Set_Text_colour(WHITE);
    else if ((cOilTemperature >= 90) && (cOilTemperature <= 99))
        tft.Set_Text_colour(YELLOW);
    else if ((cOilTemperature >= 100) && (cOilTemperature <= 109))
        tft.Set_Text_colour(ORANGE);
    else if (cOilTemperature >= 110)
        tft.Set_Text_colour(RED);
}

void vCoolantTempColor(int8_t cCoolantTemperature)
{
    tft.Set_Text_Back_colour(BLACK);
    tft.Set_Text_Size(2);

    if (cCoolantTemperature <= 69)
        tft.Set_Text_colour(CYAN);
    else if ((cCoolantTemperature >= 70) && (cCoolantTemperature <= 94))
        tft.Set_Text_colour(WHITE);
    else if ((cCoolantTemperature >= 95) && (cCoolantTemperature <= 99))
        tft.Set_Text_colour(YELLOW);
    else if ((cCoolantTemperature >= 100) && (cCoolantTemperature <= 104))
        tft.Set_Text_colour(ORANGE);
    else if (cCoolantTemperature >= 105)
        tft.Set_Text_colour(RED);
}

bool bInitBluetooth(void)
{
    if (!btStart())
    {
        DEBUG_PRINTS("Failed to initialize controller");
        return false;
    }

    if (esp_bluedroid_init() != ESP_OK)
    {
        DEBUG_PRINTS("Failed to initialize bluedroid");
        return false;
    }

    if (esp_bluedroid_enable() != ESP_OK)
    {
        DEBUG_PRINTS("Failed to enable bluedroid");
        return false;
    }
    return true;
}

void vSetupDisplay(void)
{
    tft.init();

    tft.fillScreen(BLACK);
    tft.Set_Text_Mode(0);

    for (register uint8_t rotation = 0; rotation < 4; rotation++)
    {
        // Rotate the screen to guarantee a full empty screen
        tft.setRotation(rotation);
        tft.fillScreen(BLACK);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    tft.setRotation(3);
    tft.Set_Text_colour(WHITE);
    tft.Set_Text_Back_colour(BLACK);
}

void vUnpairDevices(void)
{
    uint8_t ui8PairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
    int32_t i32Count = esp_bt_gap_get_bond_device_num();

    tft.Print_String("\tLooking for paired\n devices...", LEFT, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if (i32Count > 0)
    {
        char cStrCount[19];
        char cPlural[8];

        if (i32Count == 1)
            sprintf(cPlural, "device");
        else
            sprintf(cPlural, "devices");

        sprintf(cStrCount, "\n\t%d %s paired", i32Count, cPlural);
        tft.Print_String(cStrCount, LEFT, tft.Get_Text_Y_Cousur());

        esp_err_t tError = esp_bt_gap_get_bond_device_list(&i32Count, ui8PairedDeviceBtAddr);

        if (tError == ESP_OK)
        {
            if (i32Count >= PAIR_MAX_DEVICES)
            {
                tft.Print_String("\n\tMaximum reached", LEFT, tft.Get_Text_Y_Cousur());
                vTaskDelay(1000 / portTICK_PERIOD_MS);

                tft.Print_String("\n\tUnpairing all...", LEFT, tft.Get_Text_Y_Cousur());
                vTaskDelay(1000 / portTICK_PERIOD_MS);

                for (register uint8_t i = 0; i < i32Count; i++)
                    esp_bt_gap_remove_bond_device(ui8PairedDeviceBtAddr[i]);
                vTaskDelay(1000 / portTICK_PERIOD_MS);

                tft.Print_String("\n\tOK", LEFT, tft.Get_Text_Y_Cousur());
            }
        }
    }
    else
        tft.Print_String("\n\tNo devices paired", LEFT, tft.Get_Text_Y_Cousur());

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void vSetupELM(void)
{
    tft.Print_String("\n\tSearching for OBDII", LEFT, tft.Get_Text_Y_Cousur());

    SerialBT.begin("ESP32", true);

    while (!SerialBT.connect("OBDII"))
        ;
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    while (!myELM327.begin(SerialBT, '0'))
        ;
    tft.Print_String("\n\tConnected to OBDII", LEFT, tft.Get_Text_Y_Cousur());
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    SerialBT.println("AT Z"); // Reset All
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void vSetupTouchPad(void)
{
    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    touch_pad_config((touch_pad_t)0, TOUCH_THRESH_NO_USE);
    touch_pad_config((touch_pad_t)2, TOUCH_THRESH_NO_USE);

    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
}

void vHomeScreen(void)
{
    tft.fillScreen(BLACK);
    tft.Set_Text_Size(1);

    tft.Set_Text_colour(MAGENTA);
    tft.Print_String("BOOST!", CENTER, 43);

    tft.Set_Text_colour(WHITE);
    tft.Print_String("IAT", 3, 84);
    tft.Print_String("Oil T", 69, 84);
    tft.Print_String("ECT", 3, 120);
    tft.Print_String("HPFP", 97, 120);
    tft.Print_String("Timing", CENTER, 120);

    tft.Set_Draw_color(WHITE);
    tft.Draw_Rectangle(0, 0, 129, 129); // Outer Line
    tft.Draw_Line(0, 52, 129, 52);      // Horizontal line 1
    tft.Draw_Line(0, 94, 129, 94);      // Horizontal line 2
    tft.Draw_Line(65, 52, 65, 94);      // Vertical line 1
    tft.Draw_Line(43, 94, 43, 129);     // Vertical line 2
    tft.Draw_Line(86, 94, 86, 129);     // Vertical line 3
}

void vError(void)
{
#ifdef DEBUG
    int8_t i8Status = myELM327.status;

    if (i8Status == ELM_NO_RESPONSE)
        printf("ERROR: ELM_NO_RESPONSE\n");
    else if (i8Status == ELM_BUFFER_OVERFLOW)
        printf("ERROR: ELM_BUFFER_OVERFLOW\n");
    else if (i8Status == ELM_UNABLE_TO_CONNECT)
        printf("ERROR: ELM_UNABLE_TO_CONNECT\n");
    else if (i8Status == ELM_NO_DATA)
        printf("ERROR: ELM_NO_DATA\n");
    else if (i8Status == ELM_STOPPED)
        printf("ERROR: ELM_STOPPED\n");
    else if (i8Status == ELM_TIMEOUT)
        printf("ERROR: ELM_TIMEOUT\n");
#endif

    SerialBT.println("AT"); // Stop
    vWaitForOK();

    SerialBT.println("AT Z"); // Reset All
    vWaitForOK();
}

void vGetBoost(void *pvParameters)
{
    static uint8_t ucBoost = 0;
    static uint8_t ucBoostMaxValue = 0;

    for (;;)
    {
        xQueuePeek(xQueueBoostMaxValue, &ucBoostMaxValue, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            ucBoost = myELM327.manifoldPressure();

            if (myELM327.status == ELM_SUCCESS)
            {
                xQueueOverwrite(xQueueBoost, &ucBoost);

                if (ucBoost > ucBoostMaxValue)
                {
                    ucBoostMaxValue = ucBoost;
                    xQueueOverwrite(xQueueBoostMaxValue, &ucBoostMaxValue);
                }
            }
            else
            {
                DEBUG_PRINTS("Boost ");
                vError();
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Get Boost: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vGetIAT(void *pvParameters)
{
    static int8_t cIAT = 0;
    static int8_t cIATMaxValue = -127;

    for (;;)
    {
        xQueuePeek(xQueueIATMaxValue, &cIATMaxValue, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            cIAT = myELM327.intakeAirTemp();

            if (myELM327.status == ELM_SUCCESS)
            {
                xQueueOverwrite(xQueueIAT, &cIAT);

                if (cIAT > cIATMaxValue)
                {
                    cIATMaxValue = cIAT;
                    xQueueOverwrite(xQueueIATMaxValue, &cIATMaxValue);
                }
            }
            else
            {
                DEBUG_PRINTS("IAT ");
                vError();
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Get IAT: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void vGetOilAndCoolantTemp(void *pvParameters)
{
    int8_t cOilTemp = 0;
    int8_t cCoolant = 0;
    static int8_t cOilTemperatureMaxValue = -127;
    static int8_t cCoolantTemperatureMaxValue = -127;

    for (;;)
    {
        xQueuePeek(xQueueOilMaxValue, &cOilTemperatureMaxValue, portMAX_DELAY);
        xQueuePeek(xQueueCoolantMaxValue, &cCoolantTemperatureMaxValue, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            char cPayload[64];

            for (register uint8_t i = 0; (i <= sizeof(cPayload) - 1); i++)
                cPayload[i] = '\0';

            SerialBT.println("AT CAF 0"); // CAN Auto Formatting Off for non standard OBD
            vWaitForOK();

            SerialBT.println("AT CF 488"); // CAN Filter 488
            vWaitForOK();

            SerialBT.println("AT MR 04"); // Read header 4xx and gives time to receive
            vTaskDelay(10 / portTICK_PERIOD_MS);

            if (SerialBT.available())
                SerialBT.readBytes(cPayload, sizeof(cPayload) - 1);

            SerialBT.println("AT"); // Stop
            vWaitForOK();

            SerialBT.println("AT CAF 1"); // Required for OBD standard PIDs
            vWaitForOK();

            SerialBT.println("AT CF 7E8"); // CAN Filter 7E8 (OBD standard, 7E0 to 7E8)
            vWaitForOK();

            if ((myELM327.status == ELM_SUCCESS) && strstr(cPayload, "AT CAF 0") && strstr(cPayload, "AT CF 488") && strstr(cPayload, "AT MR 04"))
            {
                for (register uint8_t i = 0; i <= 3; i++)
                {
                    uint8_t k = 38;

                    if (i == 1)
                        k = 39;
                    else if (i == 2)
                        k = 53;
                    else if (i == 3)
                        k = 54;

                    if (cPayload[k] >= '0' && cPayload[k] <= '9')
                        cPayload[k] -= 48;
                    else if (cPayload[k] >= 'A' && cPayload[k] <= 'F')
                        cPayload[k] -= 55;
                }

                uint8_t ucTempOilTemp = (cPayload[53] << 4) | cPayload[54];
                cOilTemp = ucTempOilTemp - 40;

                uint8_t ucTempCoolant = (cPayload[38] << 4) | cPayload[39];
                cCoolant = ucTempCoolant - 40;

                xQueueOverwrite(xQueueOil, &cOilTemp);
                xQueueOverwrite(xQueueCoolant, &cCoolant);

                if (cOilTemp > cOilTemperatureMaxValue)
                {
                    cOilTemperatureMaxValue = cOilTemp;
                    xQueueOverwrite(xQueueOilMaxValue, &cOilTemperatureMaxValue);
                }

                if (cCoolant > cCoolantTemperatureMaxValue)
                {
                    cCoolantTemperatureMaxValue = cCoolant;
                    xQueueOverwrite(xQueueCoolantMaxValue, &cCoolantTemperatureMaxValue);
                }
            }
            else
            {
                DEBUG_PRINTS("Oil Temp and Coolant ");
                vError();
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Get Oil and Coolant Temp: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(15000 / portTICK_PERIOD_MS);
    }
}

void vGetTimingAdvance(void *pvParameters)
{
    static int8_t cTimingAdvance = 0;

    for (;;)
    {
        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            cTimingAdvance = myELM327.timingAdvance();

            if (myELM327.status == ELM_SUCCESS)
                xQueueOverwrite(xQueueTimingAdvance, &cTimingAdvance);
            else
            {
                DEBUG_PRINTS("Timing ");
                vError();
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Get Timing Advance: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vGetHPFPPressure(void *pvParameters)
{
    static uint16_t ui16HPFPPressure = 0;

    for (;;)
    {
        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            ui16HPFPPressure = myELM327.fuelRailGuagePressure();

            if (myELM327.status == ELM_SUCCESS)
                xQueueOverwrite(xQueueHPFPPressure, &ui16HPFPPressure);
            else
            {
                DEBUG_PRINTS("HPFP ");
                vError();
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Get HPFP: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintBoost(void *pvParameters)
{
    static uint8_t ucReceivedBoost = 99;
    static uint8_t ucReceivedBoostMaxValue = 99;

    for (;;)
    {
        xQueuePeek(xQueueBoost, &ucReceivedBoost, portMAX_DELAY);
        xQueuePeek(xQueueBoostMaxValue, &ucReceivedBoostMaxValue, portMAX_DELAY);

        float fReceivedBoost = ((float)ucReceivedBoost / 100) - 1;
        float fReceivedBoostMaxValue = ((float)ucReceivedBoostMaxValue / 100) - 1;

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if ((ucReceivedBoost >= 1) && (ucReceivedBoost <= 254))
            {
                if ((ucReceivedBoost >= 1) && (ucReceivedBoost <= 100))
                    fReceivedBoost = 0;

                vBoostColor(ucReceivedBoost);
                tft.Print_Number_Float(fReceivedBoost, 2, CENTER, 3, '.', 4, ' ');

                vBoostColor(ucReceivedBoostMaxValue);
                tft.Set_Text_Size(1);
                if (ucReceivedBoostMaxValue > BOOST_RESET_VALUE)
                    tft.Print_Number_Float(fReceivedBoostMaxValue, 2, 103, 43, '.', 4, ' ');
                else
                    tft.Print_String("   ", 103, 43);

                DEBUG_PRINTSS("Boost: %.2f\n", fReceivedBoost);
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Print Boost: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintIAT(void *pvParameters)
{
    static int8_t cReceivedIAT = 0;
    static int8_t cReceivedIATMaxValue = 0;

    for (;;)
    {
        xQueuePeek(xQueueIAT, &cReceivedIAT, portMAX_DELAY);
        xQueuePeek(xQueueIATMaxValue, &cReceivedIATMaxValue, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if ((cReceivedIAT >= -39) && (cReceivedIAT <= 126))
            {
                vIATColor(cReceivedIAT);
                tft.Print_Number_Int(cReceivedIAT, 7, 59, 4, ' ', 10);

                vIATColor(cReceivedIATMaxValue);
                tft.Set_Text_Size(1);
                if (cReceivedIATMaxValue > TEMP_RESET_VALUE)
                    tft.Print_Number_Int(cReceivedIATMaxValue, 45, 84, 4, ' ', 10);
                else
                    tft.Print_String("   ", 45, 84);

                DEBUG_PRINTSS("IAT: %d\n", cReceivedIAT);
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Print IAT: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintOilAndCoolantTemp(void *pvParameters)
{
    static int8_t cReceivedOilTemperature = 0;
    static int8_t cReceivedCoolantTemperature = 0;
    static int8_t cReceivedOilTemperatureMaxValue = 0;
    static int8_t cReceivedCoolantTemperatureMaxValue = 0;

    for (;;)
    {
        xQueuePeek(xQueueOil, &cReceivedOilTemperature, portMAX_DELAY);
        xQueuePeek(xQueueCoolant, &cReceivedCoolantTemperature, portMAX_DELAY);
        xQueuePeek(xQueueOilMaxValue, &cReceivedOilTemperatureMaxValue, portMAX_DELAY);
        xQueuePeek(xQueueCoolantMaxValue, &cReceivedCoolantTemperatureMaxValue, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if ((cReceivedOilTemperature >= -39) && (cReceivedOilTemperature <= 126))
            {
                vOilTempColor(cReceivedOilTemperature);
                tft.Print_Number_Int(cReceivedOilTemperature, 72, 59, 4, ' ', 10);

                vOilTempColor(cReceivedOilTemperatureMaxValue);
                tft.Set_Text_Size(1);
                if (cReceivedOilTemperatureMaxValue > TEMP_RESET_VALUE)
                    tft.Print_Number_Int(cReceivedOilTemperatureMaxValue, 109, 84, 4, ' ', 10);
                else
                    tft.Print_String("   ", 109, 84);

                DEBUG_PRINTSS("Oil Temp: %d\n", cReceivedOilTemperature);
            }

            if ((cReceivedCoolantTemperature >= -39) && (cReceivedCoolantTemperature <= 126))
            {
                vCoolantTempColor(cReceivedCoolantTemperature);
                tft.Print_Number_Int(cReceivedCoolantTemperature, 5, 100, 4, ' ', 10);

                vCoolantTempColor(cReceivedCoolantTemperatureMaxValue);
                tft.Set_Text_Size(1);
                if (cReceivedCoolantTemperatureMaxValue > TEMP_RESET_VALUE)
                    tft.Print_Number_Int(cReceivedCoolantTemperatureMaxValue, 23, 120, 4, ' ', 10);
                else
                    tft.Print_String("   ", 23, 120);

                DEBUG_PRINTSS("Coolant: %d\n", cReceivedCoolantTemperature);
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Print Oil and Coolant Temp: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintTimingAdvance(void *pvParameters)
{
    static int8_t cReceivedTimingAdvance = 0;

    for (;;)
    {
        xQueuePeek(xQueueTimingAdvance, &cReceivedTimingAdvance, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if ((cReceivedTimingAdvance >= -63) && (cReceivedTimingAdvance <= 63))
            {
                tft.Set_Text_colour(WHITE);
                tft.Set_Text_Back_colour(BLACK);
                tft.Set_Text_Size(2);

                tft.Print_Number_Int(cReceivedTimingAdvance, 48, 100, 4, ' ', 10);

                DEBUG_PRINTSS("Timing: %d\n", cReceivedTimingAdvance);
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Print Timing Advance: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintHPFPPressure(void *pvParameters)
{
    static uint16_t ui16ReceivedHPFPPressure = 0;

    for (;;)
    {
        xQueuePeek(xQueueHPFPPressure, &ui16ReceivedHPFPPressure, portMAX_DELAY);

        uint8_t ucReceivedHPFPPressure = ui16ReceivedHPFPPressure / 100;

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if ((ucReceivedHPFPPressure >= 1) && (ucReceivedHPFPPressure <= 254))
            {
                tft.Set_Text_colour(WHITE);
                tft.Set_Text_Back_colour(BLACK);
                tft.Set_Text_Size(2);

                tft.Print_Number_Int(ucReceivedHPFPPressure, 90, 100, 4, ' ', 10);

                DEBUG_PRINTSS("HPFP: %d\n", ui16ReceivedHPFPPressure);
            }

            xSemaphoreGive(xSemaphore);
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Print HPFP: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vTouchPadRead(void *pvParameters)
{
    static uint16_t ui16IncrementVar = 0;

    for (;;)
    {
        if ((isPressed()) && (ui16IncrementVar <= 5))
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ui16IncrementVar++;
        }
        else if (ui16IncrementVar > 0)
        {
            if (ui16IncrementVar > 50)
            {
                DEBUG_PRINTSS("Incremento: %d RESTART ESP32\n", ui16IncrementVar);

                /*
                .
                .
                .
                */
            }
            /*else if ((ui16IncrementVar > 5) && (ui16IncrementVar <= 50))*/
            if (ui16IncrementVar >= 5)
            {
                DEBUG_PRINTSS("Incremento: %d RESET VALUES\n", ui16IncrementVar);

                uint8_t ui8BoostMin = BOOST_RESET_VALUE;
                int8_t i8IATMin = TEMP_RESET_VALUE;
                int8_t i8OilMin = TEMP_RESET_VALUE;
                int8_t i8CoolantMin = TEMP_RESET_VALUE;

                xQueueOverwrite(xQueueBoostMaxValue, &ui8BoostMin);
                xQueueOverwrite(xQueueIATMaxValue, &i8IATMin);
                xQueueOverwrite(xQueueOilMaxValue, &i8OilMin);
                xQueueOverwrite(xQueueCoolantMaxValue, &i8CoolantMin);

                if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
                {
                    tft.Set_Text_Size(1);
                    tft.Print_String("    ", 103, 43);
                    tft.Print_String("   ", 45, 84);
                    tft.Print_String("   ", 109, 84);
                    tft.Print_String("   ", 23, 120);

                    xSemaphoreGive(xSemaphore);
                }
            }

            ui16IncrementVar = 0;
        }

#ifdef DEBUG_WATERMARK
        uint32_t uxHighWaterMark = uxCheckHighWaterMark();
        DEBUG_PRINTSS("Free Stack Touch Pad Read: %d\n", uxHighWaterMark);
#endif

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    static uint8_t ucBoostMaxValue = 0;
    static int8_t cIATMaxValue = -127;
    static int8_t cOilTemperatureMaxValue = -127;
    static int8_t cCoolantTemperatureMaxValue = -127;

    esp_err_t i32NVSReturn = nvs_flash_init();
    if (i32NVSReturn == ESP_ERR_NVS_NO_FREE_PAGES || i32NVSReturn == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        i32NVSReturn = nvs_flash_init();
    }
    ESP_ERROR_CHECK(i32NVSReturn);

    xSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(xSemaphore);

    xQueueBoost = xQueueCreate(1, sizeof(uint8_t));
    if (xQueueBoost == NULL)
        DEBUG_PRINTS("\nError allocating xQueueBoost");
    xQueueIAT = xQueueCreate(1, sizeof(int8_t));
    if (xQueueIAT == NULL)
        DEBUG_PRINTS("\nError allocating xQueueIAT");
    xQueueOil = xQueueCreate(1, sizeof(int8_t));
    if (xQueueOil == NULL)
        DEBUG_PRINTS("\nError allocating xQueueOil");
    xQueueCoolant = xQueueCreate(1, sizeof(int8_t));
    if (xQueueCoolant == NULL)
        DEBUG_PRINTS("\nError allocating xQueueCoolant");
    xQueueTimingAdvance = xQueueCreate(1, sizeof(int8_t));
    if (xQueueTimingAdvance == NULL)
        DEBUG_PRINTS("\nError allocating xQueueTimingAdvance");
    xQueueHPFPPressure = xQueueCreate(1, sizeof(uint16_t));
    if (xQueueHPFPPressure == NULL)
        DEBUG_PRINTS("\nError allocating xQueueHPFPPressure");

    xQueueBoostMaxValue = xQueueCreate(1, sizeof(uint8_t));
    if (xQueueBoostMaxValue == NULL)
        DEBUG_PRINTS("\nError allocating xQueueBoostMaxValue");
    xQueueIATMaxValue = xQueueCreate(1, sizeof(int8_t));
    if (xQueueIATMaxValue == NULL)
        DEBUG_PRINTS("\nError allocating xQueueIATMaxValue");
    xQueueOilMaxValue = xQueueCreate(1, sizeof(int8_t));
    if (xQueueOilMaxValue == NULL)
        DEBUG_PRINTS("\nError allocating xQueueOilMaxValue");
    xQueueCoolantMaxValue = xQueueCreate(1, sizeof(int8_t));
    if (xQueueCoolantMaxValue == NULL)
        DEBUG_PRINTS("\nError allocating xQueueCoolantMaxValue");

    xQueueOverwrite(xQueueBoostMaxValue, &ucBoostMaxValue);
    xQueueOverwrite(xQueueIATMaxValue, &cIATMaxValue);
    xQueueOverwrite(xQueueOilMaxValue, &cOilTemperatureMaxValue);
    xQueueOverwrite(xQueueCoolantMaxValue, &cCoolantTemperatureMaxValue);

    bInitBluetooth();
    vSetupDisplay();
    vUnpairDevices();
    vSetupELM();
    vSetupTouchPad();
    vHomeScreen();

    if (xTaskCreatePinnedToCore(vGetBoost, "Get Boost", 1024 * 3, NULL, 4, NULL, CORE_0) != pdPASS)
        DEBUG_PRINTS("\nError allocating Get Boost Task");
    if (xTaskCreatePinnedToCore(vGetIAT, "Get IAT", 1024 * 3, NULL, 2, NULL, CORE_0) != pdPASS)
        DEBUG_PRINTS("\nError allocating Get IAT Task");
    if (xTaskCreatePinnedToCore(vGetOilAndCoolantTemp, "Get Oil and Coolant Temp", 1024 * 3, NULL, 2, NULL, CORE_0) != pdPASS)
        DEBUG_PRINTS("\nError allocating Get Oil and Coolant Temperatures Task");
    if (xTaskCreatePinnedToCore(vGetTimingAdvance, "Get Timing (Relative to 1st Cyl)", 1024 * 3, NULL, 3, NULL, CORE_0) != pdPASS)
        DEBUG_PRINTS("\nError allocating Get Timing Advance Task");
    if (xTaskCreatePinnedToCore(vGetHPFPPressure, "Get HPFP Pressure", 1024 * 3, NULL, 3, NULL, CORE_0) != pdPASS)
        DEBUG_PRINTS("\nError allocating Get High Pressure Fuel Pump Pressure Task");

    if (xTaskCreatePinnedToCore(vPrintBoost, "Print Boost", 1024 * 3, NULL, 4, NULL, CORE_1) != pdPASS)
        DEBUG_PRINTS("\nError allocating Print Boost Task");
    if (xTaskCreatePinnedToCore(vPrintIAT, "Print IAT", 1024 * 3, NULL, 3, NULL, CORE_1) != pdPASS)
        DEBUG_PRINTS("\nError allocating Print IAT Task");
    if (xTaskCreatePinnedToCore(vPrintOilAndCoolantTemp, "Print Oil and Coolant Temp", 1024 * 3, NULL, 3, NULL, CORE_1) != pdPASS)
        DEBUG_PRINTS("\nError allocating Print Oil and Coolant Temperatures Task");
    if (xTaskCreatePinnedToCore(vPrintTimingAdvance, "Print Timing (Relative to 1st Cyl)", 1024 * 3, NULL, 3, NULL, CORE_1) != pdPASS)
        DEBUG_PRINTS("\nError allocating Print Print Timing Advance Task");
    if (xTaskCreatePinnedToCore(vPrintHPFPPressure, "Print HPFP Pressure", 1024 * 3, NULL, 3, NULL, CORE_1) != pdPASS)
        DEBUG_PRINTS("\nError allocating Print Print High Pressure Fuel Pump Pressure Task");

    if (xTaskCreatePinnedToCore(vTouchPadRead, "Touch Pad Read", 1024 * 3, NULL, 3, NULL, CORE_1) != pdPASS)
        DEBUG_PRINTS("\nError allocating Touch Pad Read Task");
}

void loop()
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
}