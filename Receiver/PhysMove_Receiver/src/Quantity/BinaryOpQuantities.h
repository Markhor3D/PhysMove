#pragma once
#include "Quantity.h"

class BinaryOpQuantity : public LoggerQuantity
{
public:
protected:
    //LoggerQuantity threshold = 0;

    // Number of dependencies on top of basic binary deps
    BinaryOpQuantity(ClassIDs classID, uint8 noOfDependencies) :LoggerQuantity(classID, noOfDependencies + 2)
    {
        // We need a sync in display and makeValue
        makeOnSamplingSpeedOnly = true;
    }
};

class BinaryAdderQuantity : public BinaryOpQuantity
{
public:
    BinaryAdderQuantity() :BinaryOpQuantity(ClassIDs::BinaryAdder, 0)
    {};
    float makeValue() override
    {
        return SafeGetQuantityValue(Dependencies[0]) + SafeGetQuantityValue(Dependencies[1]);
    }
};
class BinaryMultiplierQuantity : public BinaryOpQuantity
{
public:
    BinaryMultiplierQuantity() :BinaryOpQuantity(ClassIDs::BinaryMultiplier, 0)
    {};
    float makeValue() override
    {
        return SafeGetQuantityValue(Dependencies[0]) * SafeGetQuantityValue(Dependencies[1]);
    }

};

class BinaryDividerQuantity : public BinaryOpQuantity
{
protected:
public:
    BinaryDividerQuantity() :BinaryOpQuantity(ClassIDs ::BinaryDivider, 0)
    {};
    float makeValue() override
    {
        if (SafeGetQuantityValue(Dependencies[1]) == 0)
            return 0;
        return SafeGetQuantityValue(Dependencies[0]) / SafeGetQuantityValue(Dependencies[1]);
    }
};

class ExponentQuantity : public BinaryOpQuantity
{
protected:
    float makeValue() override
    {
        return (float)pow(SafeGetQuantityValue(Dependencies[0]), SafeGetQuantityValue(Dependencies[1]));
    }

public:
    ExponentQuantity() :BinaryOpQuantity(ClassIDs::Exponent, 0)
    {};
};



class BinaryComparatorQuantity : public BinaryOpQuantity
{
    //LoggerQuantity threshold = null;
public:
    BinaryComparatorQuantity() :BinaryOpQuantity(ClassIDs::BinaryComparator, 2)
    {
    }
    float makeValue() override
    {
        return (SafeGetQuantityValue(Dependencies[0]) > SafeGetQuantityValue(Dependencies[1])) ? SafeGetQuantityValue(Dependencies[2]) : SafeGetQuantityValue(Dependencies[3]);
    }
    float makeValueAveraged() override
    {
        // makeValue returns in the scale of dep 2 and 3. Just apply binary threshold on makeValueAveraged() 
        float avg = (SafeGetQuantityValue(Dependencies[2]) + SafeGetQuantityValue(Dependencies[3])) / 2;
        return BinaryOpQuantity::makeValueAveraged() > avg ? SafeGetQuantityValue(Dependencies[2]) : SafeGetQuantityValue(Dependencies[3]);
    }
};