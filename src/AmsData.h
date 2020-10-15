#ifndef _AMSDATA_H
#define _AMSDATA_H

#include "Arduino.h"
#include <Timezone.h>
#include "HanReader.h"

#define METER_TYPE_KAIFA 1
#define METER_TYPE_AIDON 2
#define METER_TYPE_KAMSTRUP 3
#define METER_TYPE_OMNIPOWER 4

class AmsData {
public:
    AmsData();
    AmsData(int meterType, bool substituteMissing, HanReader& hanReader);

    void apply(AmsData& other);

    unsigned long getLastUpdateMillis();

    uint32_t getPackageTimestamp();

    int getListType();

    String getListId();
    String getMeterId();
    String getMeterType();

    uint32_t getMeterTimestamp();

    uint16_t getActiveImportPower();
    uint16_t getReactiveImportPower();
    uint16_t getActiveExportPower();
    uint16_t getReactiveExportPower();

    double getL1Voltage();
    double getL2Voltage();
    double getL3Voltage();

    double getL1Current();
    double getL2Current();
    double getL3Current();

    uint32_t getActiveImportCounter();
    uint32_t getReactiveImportCounter();
    uint32_t getActiveExportCounter();
    uint32_t getReactiveExportCounter();

    bool isThreePhase();

private:
    unsigned long lastUpdateMillis = 0;
    int listType = 0;
    uint32_t packageTimestamp = 0;
    String listId, meterId, meterType;
    uint32_t meterTimestamp = 0;
    int activeImportPower = 0, reactiveImportPower = 0, activeExportPower = 0, reactiveExportPower = 0;
    double l1voltage = 0, l2voltage = 0, l3voltage = 0, l1current = 0, l2current = 0, l3current = 0;
    double activeImportCounter = 0, reactiveImportCounter = 0, activeExportCounter = 0, reactiveExportCounter = 0;
    bool threePhase = false;

    void extractFromKaifa(HanReader& hanReader, int listSize);
    void extractFromAidon(HanReader& hanReader, int listSize, bool substituteMissing);
    void extractFromKamstrup(HanReader& hanReader, int listSize, bool substituteMissing);
    void extractFromOmnipower(HanReader& hanReader, int listSize);
};

#endif
