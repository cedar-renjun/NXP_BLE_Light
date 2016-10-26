/*! *********************************************************************************
* \addtogroup Heart Rate Sensor
* @{
********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
* \file app.c
* This file is the source file for the Heart Rate Sensor application
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
/* Framework / Drivers */
#include "RNG_interface.h"
#include "Keyboard.h"
#include "Led.h"
#include "TimersManager.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "panic.h"
#include "PWR_Interface.h"
#include "PWR_Configuration.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"

/* Profile / Services */
#include "battery_interface.h"
#include "device_info_interface.h"
#include "heart_rate_interface.h"

#include "board.h"
#include "ApplMain.h"
#include "app.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define mHeartRateLowerLimit_c          (40) /* Heart beat lower limit, 8-bit value */
#define mHeartRateUpperLimit_c          (201) /* Heart beat upper limit, 8-bit value */
#define mHeartRateRange_c               (mHeartRateUpperLimit_c - mHeartRateLowerLimit_c) /* Range = [ADC16_HB_LOWER_LIMIT .. ADC16_HB_LOWER_LIMIT + ADC16_HB_DYNAMIC_RANGE] */
#define mHeartRateReportInterval_c      (1)        /* heart rate report interval in seconds  */
#define mBatteryLevelReportInterval_c   (10)        /* battery level report interval in seconds  */
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef enum
{
#if gBondingSupported_d
    fastWhiteListAdvState_c,
#endif
    fastAdvState_c,
    slowAdvState_c
}advType_t;

typedef struct advState_tag{
    bool_t      advOn;
    advType_t   advType;
}advState_t;


/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/* Host Stack Data*/
static bleDeviceAddress_t   maBleDeviceAddress;
static deviceId_t           mPeerDeviceId = gInvalidDeviceId_c;

/* Adv State */
static advState_t  mAdvState;
static uint32_t     mAdvTimeout;
static bool_t      mRestartAdv;
#if gBondingSupported_d
static bleDeviceAddress_t   maPeerDeviceAddress;
static uint8_t mcBondedDevices = 0;
static bleAddressType_t     mPeerDeviceAddressType;
#endif

/* Service Data*/
static basConfig_t      basServiceConfig = {service_battery, 0};
static disConfig_t      disServiceConfig = {service_device_info};
static hrsUserData_t    hrsUserData;
static hrsConfig_t hrsServiceConfig = {service_heart_rate, TRUE, TRUE, TRUE, gHrs_BodySensorLocChest_c, &hrsUserData};
static uint16_t cpHandles[1] = { value_hr_ctrl_point };

/* Application specific data*/
static bool_t mToggle16BitHeartRate = FALSE;
static bool_t mContactStatus = TRUE;
static tmrTimerID_t mAdvTimerId;
static tmrTimerID_t mMeasurementTimerId;
static tmrTimerID_t mBatteryMeasurementTimerId;

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/

/* Gatt and Att callbacks */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent);
static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent);
static void BleApp_GattServerCallback (deviceId_t deviceId, gattServerEvent_t* pServerEvent);
static void BleApp_Config();

/* Timer Callbacks */
static void AdvertisingTimerCallback (void *);
static void TimerMeasurementCallback (void *);
static void BatteryMeasurementTimerCallback (void *);

static void BleApp_Advertise(void);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{
    /* Initialize application support for drivers */
    BOARD_InitAdc();

    LED_PWM_Init();
}

/*! *********************************************************************************
* \brief    Starts the BLE application.
*
********************************************************************************** */
void BleApp_Start(void)
{
    /* Device is not connected and not advertising*/
    if (!mAdvState.advOn)
    {
#if gBondingSupported_d
        if (mcBondedDevices > 0)
        {
            mAdvState.advType = fastWhiteListAdvState_c;
        }
        else
        {
#endif
            mAdvState.advType = fastAdvState_c;
#if gBondingSupported_d
        }
#endif
        BleApp_Advertise();
    }

#if (cPWR_UsePowerDownMode)
    PWR_ChangeDeepSleepMode(1); /* MCU=LLS3, LL=DSM, wakeup on GPIO/LL */
    PWR_AllowDeviceToSleep();
#endif
}

/*! *********************************************************************************
* \brief        Handles keyboard events.
*
* \param[in]    events    Key event structure.
********************************************************************************** */
void BleApp_HandleKeys(key_event_t events)
{
    static uint8_t duty = 10;
    static uint8_t init = 0;
    switch (events)
    {
        case gKBD_EventPressPB1_c:
        {
            if (mPeerDeviceId == gInvalidDeviceId_c)
            {
                BleApp_Start();
            }

            if(init == 0)
            {
                init = 1;
                //LED_PWM_Init();
                //LED_W_Set(50);
            }
            else
            {
                if(duty <= 90)
                {
                    duty += 10;
                }
                else
                {
                    duty = 0;
                }
                //LED_W_Set(duty);
                //LED_B_Set(duty);
                //LED_G_Set(duty);
                //LED_R_Set(duty);
            }

            //Led1Toggle();

            break;
        }
        case gKBD_EventPressPB2_c:
        {
            mToggle16BitHeartRate = (mToggle16BitHeartRate)?FALSE:TRUE;

            Led1Toggle();

            //LED_W_Set(duty);
        }
        break;
        case gKBD_EventLongPB1_c:
        {
            if (mPeerDeviceId != gInvalidDeviceId_c)
            {
                Gap_Disconnect(mPeerDeviceId);
            }
            break;
        }
        case gKBD_EventLongPB2_c:
        {
            mContactStatus = mContactStatus?FALSE:TRUE;
            Hrs_SetContactStatus(service_heart_rate, mContactStatus);
            break;
        }
    default:
         break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE generic callback.
*
* \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
********************************************************************************** */
void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent)
{
    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:
        {
            BleApp_Config();
        }
        break;

        case gPublicAddressRead_c:
        {
            /* Use address read from the controller */
            FLib_MemCpyReverseOrder(maBleDeviceAddress, pGenericEvent->eventData.aAddress, sizeof(bleDeviceAddress_t));
        }
        break;

        case gAdvertisingDataSetupComplete_c:
        {
        }
        break;

        case gAdvertisingParametersSetupComplete_c:
        {
            App_StartAdvertising(BleApp_AdvertisingCallback, BleApp_ConnectionCallback);
        }
        break;

        case gAdvertisingSetupFailed_c:
        case gInternalError_c:
        {
            panic(0,0,0,0);
        }
        break;

        default:
            break;
    }
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief        Configures BLE Stack after initialization. Usually used for
*               configuring advertising, scanning, white list, services, et al.
*
********************************************************************************** */
static void BleApp_Config()
{
    /* Read public address from controller */
    Gap_ReadPublicDeviceAddress();

    /* Register for callbacks*/
    GattServer_RegisterHandlesForWriteNotifications(NumberOfElements(cpHandles), cpHandles);
    App_RegisterGattServerCallback(BleApp_GattServerCallback);

    /* Register security requirements */
#if gUseServiceSecurity_d
    Gap_RegisterDeviceSecurityRequirements(&deviceSecurityRequirements);
#endif

    /* Set local passkey */
#if gBondingSupported_d
    Gap_SetLocalPasskey(gPasskeyValue_c);
#endif

    /* Setup Advertising and scanning data */
    Gap_SetAdvertisingData(&gAppAdvertisingData, &gAppScanRspData);

    /* Populate White List if bonding is supported */
#if gBondingSupported_d
    bleDeviceAddress_t aBondedDevAdds[gcGapMaximumBondedDevices_d];
    bleResult_t result = Gap_GetBondedStaticAddresses(aBondedDevAdds, gcGapMaximumBondedDevices_d, &mcBondedDevices);

    if (gBleSuccess_c == result && mcBondedDevices > 0)
    {
        for (uint8_t i = 0; i < mcBondedDevices; i++)
        {
            Gap_AddDeviceToWhiteList(gBleAddrTypePublic_c, aBondedDevAdds[i]);
        }
    }
#endif

    mAdvState.advOn = FALSE;

    /* Start services */
    hrsServiceConfig.sensorContactDetected = mContactStatus;
#if gHrs_EnableRRIntervalMeasurements_d
    hrsServiceConfig.pUserData->pStoredRrIntervals = MEM_BufferAlloc(sizeof(uint16_t) * gHrs_NumOfRRIntervalsRecorded_c);
#endif
    Hrs_Start(&hrsServiceConfig);

    basServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_Start(&basServiceConfig);

    /* Allocate application timers */
    mAdvTimerId = TMR_AllocateTimer();
    mMeasurementTimerId = TMR_AllocateTimer();
    mBatteryMeasurementTimerId = TMR_AllocateTimer();

#if (cPWR_UsePowerDownMode)
    PWR_ChangeDeepSleepMode(3); /* MCU=LLS3, LL=IDLE, wakeup on GPIO/LL */
    PWR_AllowDeviceToSleep();
#endif
}

/*! *********************************************************************************
* \brief        Configures GAP Advertise parameters. Advertise will satrt after
*               the parameters are set.
*
********************************************************************************** */
static void BleApp_Advertise(void)
{
    switch (mAdvState.advType)
    {
#if gBondingSupported_d
        case fastWhiteListAdvState_c:
        {
            gAdvParams.minInterval = gFastConnMinAdvInterval_c;
            gAdvParams.maxInterval = gFastConnMaxAdvInterval_c;
            gAdvParams.filterPolicy = (gapAdvertisingFilterPolicyFlags_t)(gProcessConnWhiteListFlag_c | gProcessScanWhiteListFlag_c);
            mAdvTimeout = gFastConnWhiteListAdvTime_c;
        }
        break;
#endif
        case fastAdvState_c:
        {
            gAdvParams.minInterval = gFastConnMinAdvInterval_c;
            gAdvParams.maxInterval = gFastConnMaxAdvInterval_c;
            gAdvParams.filterPolicy = gProcessAll_c;
            mAdvTimeout = gFastConnAdvTime_c - gFastConnWhiteListAdvTime_c;
        }
        break;

        case slowAdvState_c:
        {
            gAdvParams.minInterval = gReducedPowerMinAdvInterval_c;
            gAdvParams.maxInterval = gReducedPowerMinAdvInterval_c;
            gAdvParams.filterPolicy = gProcessAll_c;
            mAdvTimeout = gReducedPowerAdvTime_c;
        }
        break;
    }

    /* Set advertising parameters */
    Gap_SetAdvertisingParameters(&gAdvParams);
}

/*! *********************************************************************************
* \brief        Handles BLE Advertising callback from host stack.
*
* \param[in]    pAdvertisingEvent    Pointer to gapAdvertisingEvent_t.
********************************************************************************** */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            mAdvState.advOn = !mAdvState.advOn;

            if (!mAdvState.advOn && mRestartAdv)
            {
                BleApp_Advertise();
                break;
            }

#if (cPWR_UsePowerDownMode)
            if(!mAdvState.advOn)
            {
                Led1Off();
                PWR_ChangeDeepSleepMode(3);
                PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
                PWR_AllowDeviceToSleep();
            }
            else
            {
                PWR_ChangeDeepSleepMode(1);
                /* Start advertising timer */
                TMR_StartLowPowerTimer(mAdvTimerId,gTmrLowPowerSecondTimer_c,
                         TmrSeconds(mAdvTimeout), AdvertisingTimerCallback, NULL);
                Led1On();
            }
#else
            LED_StopFlashingAllLeds();
            Led1Flashing();

            if(!mAdvState.advOn)
            {
                Led2Flashing();
                Led3Flashing();
                Led4Flashing();
            }
            else
            {
                TMR_StartLowPowerTimer(mAdvTimerId,gTmrLowPowerSecondTimer_c,
                        TmrSeconds(mAdvTimeout), AdvertisingTimerCallback, NULL);
            }
#endif
        }
        break;

        case gAdvertisingCommandFailed_c:
        {
            panic(0,0,0,0);
        }
        break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE Connection callback from host stack.
*
* \param[in]    peerDeviceId        Peer device ID.
* \param[in]    pConnectionEvent    Pointer to gapConnectionEvent_t.
********************************************************************************** */
static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            mPeerDeviceId = peerDeviceId;

            /* Advertising stops when connected */
            mAdvState.advOn = FALSE;

#if gBondingSupported_d
            /* Copy peer device address information */
            mPeerDeviceAddressType = pConnectionEvent->eventData.connectedEvent.peerAddressType;
            FLib_MemCpy(maPeerDeviceAddress, pConnectionEvent->eventData.connectedEvent.peerAddress, sizeof(bleDeviceAddress_t));
#endif
#if gUseServiceSecurity_d
            {
                bool_t isBonded = FALSE ;

                if (gBleSuccess_c == Gap_CheckIfBonded(peerDeviceId, &isBonded) &&
                    FALSE == isBonded)
                {
                      Gap_SendSlaveSecurityRequest(peerDeviceId, TRUE, gSecurityMode_1_Level_3_c);
                }
            }
#endif
            /* Subscribe client*/
            Bas_Subscribe(peerDeviceId);
            Hrs_Subscribe(peerDeviceId);

#if (!cPWR_UsePowerDownMode)
            /* UI */
            LED_StopFlashingAllLeds();
            Led1On();
#endif

            /* Stop Advertising Timer*/
            mAdvState.advOn = FALSE;
            TMR_StopTimer(mAdvTimerId);

            /* Start measurements */
            //TMR_StartLowPowerTimer(mMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
            //           TmrSeconds(mHeartRateReportInterval_c), TimerMeasurementCallback, NULL);

            /* Start battery measurements */
            TMR_StartLowPowerTimer(mBatteryMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
                       TmrSeconds(mBatteryLevelReportInterval_c), BatteryMeasurementTimerCallback, NULL);

#if (cPWR_UsePowerDownMode)
    PWR_SetDeepSleepTimeInMs(900);
    PWR_ChangeDeepSleepMode(1);
    PWR_AllowDeviceToSleep();
#endif
        }
        break;

        case gConnEvtDisconnected_c:
        {
            /* Unsubscribe client */
            Bas_Unsubscribe();
            Hrs_Unsubscribe();

            mPeerDeviceId = gInvalidDeviceId_c;

            TMR_StopTimer(mMeasurementTimerId);
            TMR_StopTimer(mBatteryMeasurementTimerId);

#if (cPWR_UsePowerDownMode)
            /* UI */
            Led1Off();

            /* Go to sleep */
            PWR_ChangeDeepSleepMode(3); /* MCU=LLS3, LL=IDLE, wakeup on swithes/LL */
            PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
            PWR_AllowDeviceToSleep();
#else
            if (pConnectionEvent->eventData.disconnectedEvent.reason == gHciConnectionTimeout_c)
            {
                /* Link loss detected*/
                BleApp_Start();
            }
            else
            {
              /* Connection was terminated by peer or application */
                BleApp_Start();
            }
#endif
        }
        break;

#if gBondingSupported_d
        case gConnEvtKeysReceived_c:
        {
            /* Copy peer device address information when IRK is used */
            if (pConnectionEvent->eventData.keysReceivedEvent.pKeys->aIrk != NULL)
            {
                mPeerDeviceAddressType = pConnectionEvent->eventData.keysReceivedEvent.pKeys->addressType;
                FLib_MemCpy(maPeerDeviceAddress, pConnectionEvent->eventData.keysReceivedEvent.pKeys->aAddress, sizeof(bleDeviceAddress_t));
            }
        }
        break;

        case gConnEvtPairingComplete_c:
        {
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful &&
                pConnectionEvent->eventData.pairingCompleteEvent.pairingCompleteData.withBonding)
            {
                /* If a bond is created, write device address in controller’s White List */
                Gap_AddDeviceToWhiteList(mPeerDeviceAddressType, maPeerDeviceAddress);
            }
        }
        break;

        case gConnEvtPairingRequest_c:
        {
            gPairingParameters.centralKeys = pConnectionEvent->eventData.pairingEvent.centralKeys;
            Gap_AcceptPairingRequest(peerDeviceId, &gPairingParameters);
        }
        break;

        case gConnEvtKeyExchangeRequest_c:
        {
            gapSmpKeys_t sentSmpKeys = gSmpKeys;

            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gLtk_c))
            {
                sentSmpKeys.aLtk = NULL;
                /* When the LTK is NULL EDIV and Rand are not sent and will be ignored. */
            }

            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gIrk_c))
            {
                sentSmpKeys.aIrk = NULL;
                /* When the IRK is NULL the Address and Address Type are not sent and will be ignored. */
            }

            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gCsrk_c))
            {
                sentSmpKeys.aCsrk = NULL;
            }

            Gap_SendSmpKeys(peerDeviceId, &sentSmpKeys);
            break;
        }

        case gConnEvtLongTermKeyRequest_c:
        {
            if (pConnectionEvent->eventData.longTermKeyRequestEvent.ediv == gSmpKeys.ediv &&
                pConnectionEvent->eventData.longTermKeyRequestEvent.randSize == gSmpKeys.cRandSize)
            {
                /* EDIV and RAND both matched */
                Gap_ProvideLongTermKey(peerDeviceId, gSmpKeys.aLtk, gSmpKeys.cLtkSize);
            }
            else
            /* EDIV or RAND size did not match */
            {
                Gap_DenyLongTermKey(peerDeviceId);
            }
        }
        break;
#endif

    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles GATT server callback from host stack.
*
* \param[in]    deviceId        Peer device ID.
* \param[in]    pServerEvent    Pointer to gattServerEvent_t.
********************************************************************************** */
static void BleApp_GattServerCallback (deviceId_t deviceId, gattServerEvent_t* pServerEvent)
{
    uint16_t handle;
    uint8_t status;

    switch (pServerEvent->eventType)
    {
        case gEvtAttributeWritten_c:
        {
            handle = pServerEvent->eventData.attributeWrittenEvent.handle;
            status = gAttErrCodeNoError_c;

            if (handle == value_hr_ctrl_point)
            {
                //status = Hrs_ControlPointHandler(&hrsUserData, pServerEvent->eventData.attributeWrittenEvent.aValue[0]);

                if(4 == pServerEvent->eventData.attributeWrittenEvent.cValueLength)
                {
                    LED_W_Set(pServerEvent->eventData.attributeWrittenEvent.aValue[0]);
                    LED_B_Set(pServerEvent->eventData.attributeWrittenEvent.aValue[1]);
                    LED_G_Set(pServerEvent->eventData.attributeWrittenEvent.aValue[2]);
                    LED_R_Set(pServerEvent->eventData.attributeWrittenEvent.aValue[3]);

                    Hrs_UpdateLedDatabase (service_heart_rate, &(pServerEvent->eventData.attributeWrittenEvent.aValue[0]));

                    status = gAttErrCodeNoError_c;
                }
                else
                {
                    status = gAttErrCodeInvalidAttributeValueLength_c;
                }
            }

            GattServer_SendAttributeWrittenStatus(deviceId, handle, status);
        }
        break;

    case gEvtCharacteristicCccdWritten_c:
        {
            //while(1);
        }
    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles advertising timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void AdvertisingTimerCallback(void * pParam)
{
    /* Stop and restart advertising with new parameters */
    Gap_StopAdvertising();
    switch (mAdvState.advType)
    {
#if gBondingSupported_d
        case fastWhiteListAdvState_c:
        {
            mAdvState.advType = fastAdvState_c;
            mRestartAdv = TRUE;
        }
        break;
#endif
        case fastAdvState_c:
        {
            mAdvState.advType = slowAdvState_c;
            mRestartAdv = TRUE;
        }
        break;

        default:
        {
            mRestartAdv = FALSE;
        }
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles measurement timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void TimerMeasurementCallback(void * pParam)
{
    uint16_t hr = BOARD_GetPotentiometerLevel();
    hr = (hr * mHeartRateRange_c) >> 12;

#if gHrs_EnableRRIntervalMeasurements_d
    Hrs_RecordRRInterval(&hrsUserData, (hr & 0x0F));
    Hrs_RecordRRInterval(&hrsUserData,(hr & 0xF0));
#endif

    if (mToggle16BitHeartRate)
    {
        Hrs_RecordHeartRateMeasurement(service_heart_rate, 0x0100 + (hr & 0xFF), &hrsUserData);
    }
    else
    {
        Hrs_RecordHeartRateMeasurement(service_heart_rate, mHeartRateLowerLimit_c + hr, &hrsUserData);
    }

    Hrs_AddExpendedEnergy(&hrsUserData, 100);
}

/*! *********************************************************************************
* \brief        Handles battery measurement timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void BatteryMeasurementTimerCallback(void * pParam)
{
    basServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_RecordBatteryMeasurement(basServiceConfig.serviceHandle, basServiceConfig.batteryLevel);
}

/*! *********************************************************************************
* @}
********************************************************************************** */
