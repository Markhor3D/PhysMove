#pragma once
#include "..\BoardVersion.h"
#include "Quantity.h"
#include "..\SSD1306\Adafruit_SSD1306.h"

class MeasureLabDisplayQuantity :public LoggerQuantity
{
private:
    bool labelHasChanged = false;
    String Label;
    float ValueToDisplay = 0;
    float lastValueDisplayed = 0;
    String lastLabelDisplayed;
public:
    Adafruit_SSD1306* mlDisplay;
    MeasureLabDisplayQuantity(Adafruit_SSD1306* display):LoggerQuantity(ClassIDs::MeasureLabOnScreenDisplay, 1) {
        mlDisplay = display;
    };
    // called when quantity is active
    float makeValue() override
    {
        return SafeGetQuantityValue(Dependencies[0]);
    }
    String getPropertyValue(String& property) override
    {
        if (property == F("label")) // Screen Label
        {
            return Label;
        }
        return LoggerQuantity::getPropertyValue(property);
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("label")) // calibration parameter
        {
            labelHasChanged = true;
            Label = value;
            return true;
        }
        return LoggerQuantity::setPropertyValue(property, value);
    }
    // nothing to save by default
    void saveVariables(uint16& address) override
    {
        EEPROM2.PutString(address, Label);
    }
    // nothing to resume by default
    void resumeVariables(uint16& address) override
    {
        Label = EEPROM2.GetString(address, Label);
        if(Label != "")
            labelHasChanged = true;
    }
    // nothing to resume by default
    void resetVariables() override
    {
        firstDataWrite = true;
        labelHasChanged = false;
        Label = "";
    }
    // make output
    void setValue(float value) override
    {
        ValueToDisplay = value;
    }
    long lastDisplayedAt = 0;
    bool firstDataWrite = true;
    void DisplayLoop() {
        if (millis() - lastDisplayedAt < 100) // refresh at a reasonable rate to save CPU
            return;
        if (labelHasChanged) {
            labelHasChanged = false;
            if (lastLabelDisplayed != Label) {
                lastLabelDisplayed = Label;
                if (firstDataWrite) {
                    firstDataWrite = false;
                    mlDisplay->clearDisplay();
                }
                mlDisplay->fillRect(0, 0, 128, 42, SSD1306_BLACK);
                mlDisplay->setCursor(0, 15);
                mlDisplay->print(Label);
                mlDisplay->display();
                lastDisplayedAt = millis();
            }
        }
        if (lastValueDisplayed != ValueToDisplay) {
            lastValueDisplayed = ValueToDisplay;
            if (firstDataWrite) {
                firstDataWrite = false;
                mlDisplay->clearDisplay();
            }
            mlDisplay->fillRect(0, 45, 128, 19, SSD1306_BLACK);
            mlDisplay->setCursor(0, 62);
            mlDisplay->print(ValueToDisplay, 2);
            mlDisplay->display();
            lastDisplayedAt = millis();
        }
    }
};