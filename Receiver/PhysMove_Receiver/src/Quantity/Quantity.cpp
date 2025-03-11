#include "Quantity.h"


LoggerQuantity::LoggerQuantity(uint8 noOfDependencies)
{
    totalDependencies = noOfDependencies;
    Dependencies = new LoggerQuantity * [noOfDependencies];
    for (int i = 0; i < noOfDependencies; i++)
        Dependencies[i] = 0;
}
LoggerQuantity::~LoggerQuantity()
{
    if (Dependencies == 0)
        return;
    for (int i = 0; i < (totalDependencies & 0xF); i++)
    {
        if (Dependencies[i] == 0)
            continue;
        if (Dependencies[i]->ClassID == ClassIDs::Constant)
            delete Dependencies[i];
    }
    //delete Dependencies;
    Dependencies = 0;
}

/// <summary>
/// Abstract of "PhysQuantity". A quanity is a live state of an actual physical quanity or a derivative of that quantity.
/// </summary>
LoggerQuantity::LoggerQuantity(ClassIDs classID, uint8 noOfDependencies):LoggerQuantity(noOfDependencies)
{
    ClassID = classID;
}
    /// <summary>
    ///  set a property. Return false if property/value not supported o
    /// </summary>
bool LoggerQuantity::setPropertyValue(String& property, String& value)
{
    return false;
}
    /// <summary>
    /// Must Implement these properties        
    /// "title" // preferred title
    /// "symbol" // preferred quantity math symbol
    /// "type" // quantity type by unit.
    /// "class id" // Editor GUI type.
    /// </summary>
String LoggerQuantity::getPropertyValue(String& property)
{
    return "";
}

void LoggerQuantity::setDependency(int depIndex, LoggerQuantity& dQuantity)
{
    //Serial.print(F("Set Dep @ cid: "));
    //Serial.print(ClassID);
    //Serial.print(F(", depInd: "));
    //Serial.print(depIndex);
    //Serial.print(F(", dep cid: "));
    //Serial.print(dQuantity.ClassID);
    //Serial.print(F(", max deps: "));
    //Serial.println((totalDependencies & 0xF));
    //delay(10);

    //if (Dependencies == 0)
    //    Serial.println(F(", deps is null!!"));
    //Serial.println((totalDependencies & 0xF));
    if (depIndex < totalDependencies)
    {
        Dependencies[depIndex] = SafeAssignDependency(Dependencies[depIndex], &dQuantity);
    }
}
    // This is implemeted below ConstantQuantity class

void LoggerQuantity::setDependency(int depIndex, float value)
{
    if (Dependencies[depIndex] != 0)
    {
        if (Dependencies[depIndex]->ClassID == ClassIDs::Constant) // already have a constant q
        {
            Dependencies[depIndex]->setValue(value);
            return;
        }
    }
    // else, we need to assign a new constant q first.
    LoggerQuantity* q = new ConstantQuantity(value);
    setDependency(depIndex, *q);
}

float LoggerQuantity::getConstantDependency(int depIndex)
{
    if (depIndex >= (totalDependencies & 0xF))
        return 0;
    auto dep = Dependencies[depIndex];
    if (dep != 0)
    {
        if (dep->ClassID == ClassIDs::Constant)
            return dep->getValue();
        else
            return 0;
    }
    return 0;
}


    // don't override. Its virtual only for constant quantity
float LoggerQuantity::getValue()
{
    if (!isFresh)
    {
        cache = makeValue();
        isFresh = true;
    }
    return cache;
}
// only for output quantitites
// the actual value writer. e.g. implement LCD write (cached) according to the given value.
void LoggerQuantity::setValue(float value) { /*throw new NotImplementedException();*/ }
void LoggerQuantity::invalidate() { isFresh = false; }
float LoggerQuantity::makeValue()
{
    // the default make value.
    // must be overridden
    // this is for UI ext surrogate mechanism
    if ((totalDependencies & 0xF) > 0)
        if (Dependencies[0] != 0)
            return Dependencies[0]->getValue();
    return 0;
}
void LoggerQuantity::resetMakeValueAveraged() 
{
    averageValueSum = 0;
    avgCount = 0;
}
float LoggerQuantity::makeValueAveraged()
{
    averageValueSum += makeValue();
    avgCount++;
    if (avgCount == 1)
        return averageValueSum;
    return  averageValueSum / (float)avgCount;
}
float LoggerQuantity::getValueAveraged()
{
    // get cached value if possible
    averageValueSum += getValue();
    avgCount++;
    if (avgCount == 1)
        return averageValueSum; // cache cannot get the avged value
    else
        return averageValueSum / (float)avgCount;
}
LoggerQuantity* SafeAssignDependency(LoggerQuantity* oldQ, LoggerQuantity* newQ)
{
    if (oldQ != 0)
        if (oldQ->ClassID == ClassIDs::Constant)
            delete oldQ;
    return newQ;
}

// nothing to save by default
void LoggerQuantity::saveVariables(uint16 & address)
{}
// nothing to resume by default
void LoggerQuantity::resumeVariables(uint16 & address)
{}
// nothing to resume by default
void LoggerQuantity::resetVariables()
{}

ConstantQuantity::ConstantQuantity(float v) :LoggerQuantity(ClassIDs::Constant, 1)
{
    Value = v;
}
void ConstantQuantity::invalidate()
{

}
void ConstantQuantity::setValue(float value)
{
    Value = value;
    cache = Value; // if fresh, this will be needed;
}
float ConstantQuantity::getValue()
{
    return Value;
}
float ConstantQuantity::makeValue()
{
    return Value;
}
UserControlledLoggerQuantity::UserControlledLoggerQuantity() :ConstantQuantity(0)
{
    ClassID = ClassIDs::UserControlledQuantity;
}
void UserControlledLoggerQuantity::saveVariables(uint16& address)
{
    EEPROM2.PutFloat(address, Value);
}
void UserControlledLoggerQuantity::resumeVariables(uint16& address)
{
    EEPROM2.GetFloat(address, Value);
}
void UserControlledLoggerQuantity::resetVariables()
{
    Value = 0;
}

TimeBasedQuantity::TimeBasedQuantity(LoggerQuantity* time, ClassIDs classID, uint8 noOfDependencies):LoggerQuantity(classID, noOfDependencies)
{
    TimeQ = time;
}
