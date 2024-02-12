#include <Arduino.h>
#include <unity.h>

#include "MCP_ADC_test.h"

void setUp(void)
{

}

void tearDown(void)
{

}

int runUnityTests(void) 
{
    UNITY_BEGIN();
    /* TEST MCP_ADC */
    Serial.println("Print sth in main");
    RUN_TEST(test_MCP_ADC_sample);

    return UNITY_END();
}

void setup() 
{
    delay(500);

    runUnityTests();
}

void loop() {}