#include <Arduino.h>
#include <ELMduino.h>
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
#include <LCDWIKI_GUI.h>
#include <SSD1283A.h> // SPI SPEED = 70 MHz

#define BLACK 0x0000
#define CYAN 0x07FF
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define ORANGE 0xF900
#define RED 0xF800
#define MAGENTA 0xF81F

#define CORE_0 0
#define CORE_1 1

#define PAIR_MAX_DEVICES 3

BluetoothSerial SerialBT;
ELM327 myELM327;
SSD1283A_GUI tft(/*CS*/ 5, /*CD*/ 33, /*RST*/ 32, /*LED*/ 25);

static QueueHandle_t xQueueBoost;
static QueueHandle_t xQueueIAT;
static QueueHandle_t xQueueOil;
static QueueHandle_t xQueueCoolant;
static QueueHandle_t xQueueTimingAdvance;
static QueueHandle_t xQueueHPFPPressure;

static SemaphoreHandle_t xSemaphore;

void vWaitForOK(void)
{
    vTaskDelay(50 / portTICK_PERIOD_MS);

    /* 
    ...
    */
}

bool bInitBluetooth(void)
{
    if (!btStart())
    {
        //Serial.println("Failed to initialize controller");
        return false;
    }

    if (esp_bluedroid_init() != ESP_OK)
    {
        //Serial.println("Failed to initialize bluedroid");
        return false;
    }

    if (esp_bluedroid_enable() != ESP_OK)
    {
        //Serial.println("Failed to enable bluedroid");
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
    tft.Set_Text_Size(1);
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

    vTaskDelay(1500 / portTICK_PERIOD_MS);
    while (!myELM327.begin(SerialBT, '0'))
        ;
    tft.Print_String("\n\tConnected to OBDII", LEFT, tft.Get_Text_Y_Cousur());
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    SerialBT.println("AT Z"); // Reset All
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void vHomeScreen(void)
{
    tft.fillScreen(BLACK);

    tft.Set_Text_colour(MAGENTA);
    tft.Print_String("BOOST!", CENTER, 43);

    tft.Set_Text_colour(WHITE);
    tft.Print_String("IAT", 25, 84);
    tft.Print_String("Oil Temp", 75, 84);
    tft.Print_String("ECT", 14, 120);
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

void vGetBoost(void *pvParameters)
{
    static uint8_t ucBoost = 0;

    for (;;)
    {
        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            ucBoost = myELM327.manifoldPressure();

            xSemaphoreGive(xSemaphore);
        }

        xQueueSendToBack(xQueueBoost, &ucBoost, portMAX_DELAY);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void vGetIAT(void *pvParameters)
{
    static int8_t cIAT = 0;

    for (;;)
    {
        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            cIAT = myELM327.intakeAirTemp();

            xSemaphoreGive(xSemaphore);
        }

        xQueueSendToBack(xQueueIAT, &cIAT, portMAX_DELAY);

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vGetOilAndCoolantTemp(void *pvParameters)
{
    int8_t cOilTemp = 0;
    int8_t cCoolant = 0;

    for (;;)
    {
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
            }

            xSemaphoreGive(xSemaphore);
        }

        xQueueSendToBack(xQueueOil, &cOilTemp, portMAX_DELAY);
        xQueueSendToBack(xQueueCoolant, &cCoolant, portMAX_DELAY);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
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

            xSemaphoreGive(xSemaphore);
        }

        xQueueSendToBack(xQueueTimingAdvance, &cTimingAdvance, portMAX_DELAY);

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

            xSemaphoreGive(xSemaphore);
        }

        xQueueSendToBack(xQueueHPFPPressure, &ui16HPFPPressure, portMAX_DELAY);

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintBoost(void *pvParameters)
{
    static uint8_t ucReceivedBoost = 100;

    for (;;)
    {
        xQueueReceive(xQueueBoost, &ucReceivedBoost, portMAX_DELAY);

        float fReceivedBoost = ((float)ucReceivedBoost / 100) - 1;

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if (myELM327.status == ELM_SUCCESS)
            {
                static uint8_t ucBoostOld = 254;

                if (ucReceivedBoost != ucBoostOld)
                {
                    if (ucReceivedBoost <= 100)
                        fReceivedBoost = 0;

                    if (ucReceivedBoost <= 229)
                        tft.Set_Text_colour(WHITE);
                    else if ((ucReceivedBoost >= 230) && (ucReceivedBoost <= 239))
                        tft.Set_Text_colour(YELLOW);
                    else if ((ucReceivedBoost >= 240) && (ucReceivedBoost <= 249))
                        tft.Set_Text_colour(ORANGE);
                    else if (ucReceivedBoost >= 250)
                        tft.Set_Text_colour(RED);

                    tft.Set_Text_Back_colour(BLACK);
                    tft.Set_Text_Size(5);
                    tft.Print_Number_Float(fReceivedBoost, 2, CENTER, 3, '.', 4, ' ');

                    ucBoostOld = ucReceivedBoost;
                }
            }

            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void vPrintIAT(void *pvParameters)
{
    static int8_t cReceivedIAT = 0;

    for (;;)
    {
        xQueueReceive(xQueueIAT, &cReceivedIAT, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if (myELM327.status == ELM_SUCCESS)
            {
                if ((cReceivedIAT <= 126) && (cReceivedIAT >= -39))
                {
                    static int8_t cIATOld = 127;

                    if (cReceivedIAT != cIATOld)
                    {
                        if (cReceivedIAT <= 39)
                            tft.Set_Text_colour(WHITE);
                        else if ((cReceivedIAT >= 40) && (cReceivedIAT <= 49))
                            tft.Set_Text_colour(YELLOW);
                        else if ((cReceivedIAT >= 50) && (cReceivedIAT <= 59))
                            tft.Set_Text_colour(ORANGE);
                        else if (cReceivedIAT >= 60)
                            tft.Set_Text_colour(RED);

                        tft.Set_Text_Back_colour(BLACK);
                        tft.Set_Text_Size(3);
                        tft.Print_Number_Int(cReceivedIAT, 7, 59, 4, ' ', 10);

                        cIATOld = cReceivedIAT;
                    }
                }
            }

            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintOilAndCoolantTemp(void *pvParameters)
{
    static int8_t cReceivedOilTemperature = 0;
    static int8_t cReceivedCoolantTemperature = 0;

    for (;;)
    {
        xQueueReceive(xQueueOil, &cReceivedOilTemperature, portMAX_DELAY);
        xQueueReceive(xQueueCoolant, &cReceivedCoolantTemperature, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if ((cReceivedOilTemperature <= 126) && (cReceivedOilTemperature >= -39))
            {
                static int8_t cReceivedOilTemperatureOld = 0;

                if (cReceivedOilTemperature != cReceivedOilTemperatureOld)
                {
                    if (cReceivedOilTemperature <= 69)
                        tft.Set_Text_colour(CYAN);
                    else if ((cReceivedOilTemperature >= 70) && (cReceivedOilTemperature <= 89))
                        tft.Set_Text_colour(WHITE);
                    else if ((cReceivedOilTemperature >= 90) && (cReceivedOilTemperature <= 99))
                        tft.Set_Text_colour(YELLOW);
                    else if ((cReceivedOilTemperature >= 100) && (cReceivedOilTemperature <= 109))
                        tft.Set_Text_colour(ORANGE);
                    else if (cReceivedOilTemperature >= 110)
                        tft.Set_Text_colour(RED);

                    tft.Set_Text_Back_colour(BLACK);
                    tft.Set_Text_Size(3);
                    tft.Print_Number_Int(cReceivedOilTemperature, 72, 59, 4, ' ', 10);

                    cReceivedOilTemperatureOld = cReceivedOilTemperature;
                }
            }

            if ((cReceivedCoolantTemperature <= 126) && (cReceivedCoolantTemperature >= -39))
            {
                static int8_t cReceivedCoolantTemperatureOld = 0;

                if (cReceivedCoolantTemperature != cReceivedCoolantTemperatureOld)
                {
                    if (cReceivedCoolantTemperature <= 69)
                        tft.Set_Text_colour(CYAN);
                    else if ((cReceivedCoolantTemperature >= 70) && (cReceivedCoolantTemperature <= 89))
                        tft.Set_Text_colour(WHITE);
                    else if ((cReceivedCoolantTemperature >= 90) && (cReceivedCoolantTemperature <= 94))
                        tft.Set_Text_colour(YELLOW);
                    else if ((cReceivedCoolantTemperature >= 95) && (cReceivedCoolantTemperature <= 99))
                        tft.Set_Text_colour(ORANGE);
                    else if (cReceivedCoolantTemperature >= 100)
                        tft.Set_Text_colour(RED);

                    tft.Set_Text_Back_colour(BLACK);
                    tft.Set_Text_Size(2);
                    tft.Print_Number_Int(cReceivedCoolantTemperature, 5, 100, 4, ' ', 10);

                    cReceivedCoolantTemperatureOld = cReceivedCoolantTemperature;
                }
            }

            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void vPrintTimingAdvance(void *pvParameters)
{
    static int8_t cReceivedTimingAdvance = 0;

    for (;;)
    {
        xQueueReceive(xQueueTimingAdvance, &cReceivedTimingAdvance, portMAX_DELAY);

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if (myELM327.status == ELM_SUCCESS)
            {
                if ((cReceivedTimingAdvance <= 60) && (cReceivedTimingAdvance >= -60))
                {
                    static int8_t cReceivedTimingAdvanceOld = 127;

                    if (cReceivedTimingAdvance != cReceivedTimingAdvanceOld)
                    {
                        tft.Set_Text_colour(WHITE);
                        tft.Set_Text_Back_colour(BLACK);
                        tft.Set_Text_Size(2);
                        tft.Print_Number_Int(cReceivedTimingAdvance, 48, 100, 4, ' ', 10);

                        cReceivedTimingAdvanceOld = cReceivedTimingAdvance;
                    }
                }
            }

            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void vPrintHPFPPressure(void *pvParameters)
{
    static uint16_t ui16ReceivedHPFPPressure = 0;

    for (;;)
    {
        xQueueReceive(xQueueHPFPPressure, &ui16ReceivedHPFPPressure, portMAX_DELAY);

        uint8_t ucReceivedHPFPPressure = ui16ReceivedHPFPPressure / 100;

        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            if (myELM327.status == ELM_SUCCESS)
            {
                if ((ucReceivedHPFPPressure <= 254) && (ucReceivedHPFPPressure > 0))
                {
                    static uint8_t ucReceivedHPFPPressureOld = 254;

                    if (ucReceivedHPFPPressure != ucReceivedHPFPPressureOld)
                    {
                        tft.Set_Text_colour(WHITE);
                        tft.Set_Text_Back_colour(BLACK);
                        tft.Set_Text_Size(2);
                        tft.Print_Number_Int(ucReceivedHPFPPressure, 90, 100, 4, ' ', 10);

                        ucReceivedHPFPPressureOld = ucReceivedHPFPPressure;
                    }
                }
            }

            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    xSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(xSemaphore);

    xQueueBoost = xQueueCreate(5, sizeof(uint8_t));
    xQueueIAT = xQueueCreate(5, sizeof(int8_t));
    xQueueOil = xQueueCreate(5, sizeof(int8_t));
    xQueueCoolant = xQueueCreate(5, sizeof(int8_t));
    xQueueTimingAdvance = xQueueCreate(5, sizeof(int8_t));
    xQueueHPFPPressure = xQueueCreate(5, sizeof(uint16_t));

    bInitBluetooth();
    vSetupDisplay();
    vUnpairDevices();
    vSetupELM();
    vHomeScreen();

    xTaskCreatePinnedToCore(vGetBoost, "Get Boost Task", 2048 * 4, NULL, 4, NULL, CORE_0);
    xTaskCreatePinnedToCore(vGetIAT, "Get IAT Task", 2048 * 4, NULL, 3, NULL, CORE_0);
    xTaskCreatePinnedToCore(vGetOilAndCoolantTemp, "Get Oil and Coolant Temperatures", 2048 * 8, NULL, 2, NULL, CORE_0);
    xTaskCreatePinnedToCore(vGetTimingAdvance, "Get Timing Advance (Relative to 1st Cylinder)", 2048 * 4, NULL, 2, NULL, CORE_0);
    xTaskCreatePinnedToCore(vGetHPFPPressure, "Get High Pressure Fuel Pump Pressure", 2048 * 4, NULL, 2, NULL, CORE_0);

    xTaskCreatePinnedToCore(vPrintBoost, "Print Boost", 2048 * 4, NULL, 4, NULL, CORE_1);
    xTaskCreatePinnedToCore(vPrintIAT, "Print IAT", 2048 * 4, NULL, 3, NULL, CORE_1);
    xTaskCreatePinnedToCore(vPrintOilAndCoolantTemp, "Print Oil and Coolant Temperatures", 2048 * 8, NULL, 2, NULL, CORE_1);
    xTaskCreatePinnedToCore(vPrintTimingAdvance, "Print Timing Advance (Relative to 1st Cylinder)", 2048 * 4, NULL, 2, NULL, CORE_1);
    xTaskCreatePinnedToCore(vPrintHPFPPressure, "Print High Pressure Fuel Pump Pressure", 2048 * 4, NULL, 2, NULL, CORE_1);
}

void loop()
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
}