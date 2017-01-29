///
/// \copyright Copyright (C) 2015-2017  F1RMB, Daniel Caujolle-Bert <f1rmb.daniel@gmail.com>
///
/// \license
///  This program is free software; you can redistribute it and/or
///  modify it under the terms of the GNU General Public License
///  as published by the Free Software Foundation; either version 2
///  of the License, or (at your option) any later version.<br><br>
///  This program is distributed in the hope that it will be useful,
///  but WITHOUT ANY WARRANTY; without even the implied warranty of
///  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///  GNU General Public License for more details.<br><br>
///  You should have received a copy of the GNU General Public License
///  along with this program; if not, write to the Free Software
///  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
///
#ifndef ADSENGINE_H
#define ADSENGINE_H

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

#include "TimerOne.h"
#include "ClickEncoder.h"

///
/// \file aDSEngine.h
///    \author F1RMB, Daniel Caujolle-Bert <f1rmb.daniel@gmail.com>
///

//#define SIMU  1                  ///< Define this to run in simulation mode (temperature will raise/lower automatically, !! DO NOT PLUG ANY IRON TIP !!)
//#define CHANNEL_COUNTING     1   ///< Define this to trace channel counting, debug purpose
//#define LCD_CHANNELS_LEDS    1   ///< Define this if you want channels LEDs displayed on the LCD

static const uint8_t        CHANNEL2_ENABLE_PIN         = 13;   ///< Pin to check from if channel 2 is wired

static const uint8_t        LCD_RS_PIN                  = 7;    ///< LCD RS pin
static const uint8_t        LCD_ENABLE_PIN              = 8;    ///< LCD Enable pin
static const uint8_t        LCD_D4_PIN                  = 9;    ///< LCD D4 pin
static const uint8_t        LCD_D5_PIN                  = 10;   ///< LCD D5 pin
static const uint8_t        LCD_D6_PIN                  = 11;   ///< LCD D6 pin
static const uint8_t        LCD_D7_PIN                  = 12;   ///< LCD D7 pin

static const uint8_t        LCD_COLS                    = 16;   ///< LCD columns
static const uint8_t        LCD_ROWS                    = 2;    ///< LCD rows

static const uint8_t        ENCODER_A_PIN               = 2;    ///< Encoder A pin
static const uint8_t        ENCODER_B_PIN               = 3;    ///< Encoder B pin
static const uint8_t        ENCODER_PB_PIN              = 4;    ///< Encoder push button pin
static const uint8_t        ENCODER_STEPS_PER_NOTCH     = 4;    ///< Number of steps per notch (indent)


static const uint8_t        PWM_CHANNEL1_PIN            = 5;    ///< PWM pin of channel 1
static const uint8_t        PWM_CHANNEL2_PIN            = 6;    ///< PWM pin of channel 2

static const uint8_t        TEMP_SENSOR_CHANNEL1_PIN    = A1;   ///< Temp sensor pin of channel 1
static const uint8_t        TEMP_SENSOR_CHANNEL2_PIN    = A0;   ///< Temp sensor pin of channel 2

static const uint8_t        LED_CHANNEL1_PIN            = A2;   ///< LED pin of channel 1
static const uint8_t        LED_CHANNEL2_PIN            = A3;   ///< LED pin of channel 2


static const uint8_t        PROGRAM_VERSION_MAJOR       = 1;    ///< Major program version
static const uint8_t        PROGRAM_VERSION_MINOR       = 6;    ///< Minor program version
static const uint16_t       PROGRAM_YEAR                = 2017; ///< Release year

/// \brief Operation Mode enumeration
///
///
typedef enum
{
    OPERATION_MODE_READ,        ///< Reading values
    OPERATION_MODE_SET,         ///< Settings values
    OPERATION_MODE_UNKNOWN      ///< Unset (internal)
} OperationMode_t;


/// \brief ValueAveraging class
///
class ValueAveraging
{
    protected:
        static const uint16_t ARRAY_SIZE_MAX = 10; ///< Averaging is performed using n values

    public:
        ValueAveraging();
        ~ValueAveraging();

        void                			StackValue(int16_t value);
        int16_t             			GetValue();
        bool							SetAverage(uint16_t v);
        uint16_t						GetAverage();
        uint16_t						GetMaxAverage();
        void							ResetValues();

    private:
        int16_t           				m_values[ARRAY_SIZE_MAX]; ///< values array storage
        uint16_t             			m_offset; ///< offset in m_values[]
        uint16_t 						m_average; ///< max values used from m_values[] to build average value
};

/// \brief aDSChannel class
///
class aDSChannel
{
    public:
#ifdef SIMU
        static const int16_t        TEMPERATURE_MIN             = 10;
#else
        static const int16_t        TEMPERATURE_MIN             = 100;  ///< Minimum temperature
#endif
        static const int16_t        TEMPERATURE_MAX             = 450;  ///< Maximum temperature
        static const int16_t        TEMPERATURE_STANDBY         = 150;  ///< Standby temperature
        static const unsigned long  BLINK_UPDATE_RATE           = 400;  ///< Update rate for LED blinking, in ms
        static const int16_t        TEMPERATURE_TOLERANCE       = 3;    ///< Temperature tolerance for REACHED state, +/- 2 °C
        static const float          DEFAULT_TEMPERATURE_SLOPE   = 0.3947387545;  ///< Default slope value, used for ADC to Temp correction
        static const float          DEFAULT_TEMPERATURE_OFFSET  = 43.8279285472; ///< Default offset value, used for ADC to Temp correction

        static const int16_t        PWM_MAX_VALUE               = 150;  ///< Maximum PWM value

        /// \brief Heating State enumeration
        ///
        ///
        typedef enum
        {
            HEATING_STATE_HEATING,  ///< Heating
            HEATING_STATE_COOLING,  ///< Cooling
            HEATING_STATE_REACHED,  ///< Target temperature reached
            HEATING_STATE_STANDBY   ///< In standby mode
        } HeatingState_t;

        /// \brief Calibration values
        ///
        /// Contains Slope and Offset float values
        ///
        typedef struct
        {
            float slope;                ///< Slope value
            float offset;               ///< Offset value
        } CalibrationData_t;

    protected:
         /// \brief Our pin structure
         ///
         ///
        typedef struct
        {
            uint8_t             pin;            ///< "Arduino" pin
            uint8_t             timer;          ///< Timer of the pin
            uint8_t             mask;           ///< Bit mask of the pin
            uint8_t             port;           ///< Port of the pin
            volatile uint8_t   *outputRegister; ///< Output register of the pin
        } aPin_t;

    public:
        aDSChannel();
        virtual ~aDSChannel();

        void                setup(uint8_t, uint8_t, uint8_t);
        void                setFocus(bool);
        bool                hasFocus();
        uint16_t            getTemperature(OperationMode_t);
        bool                setTemperature(OperationMode_t, int16_t);
        bool                service(unsigned long);
        void                setStandbyMode(bool);
        bool                getStandbyMode();

        bool                isTempHasChanged();
        void                syncTempChange();
        uint8_t             updateLEDState(unsigned long);
        uint8_t             getLEDState();
        HeatingState_t      getHeatState();

        bool                isPlugged();

        void                setCalibration(float, float);
        const CalibrationData_t   getCalibration() const;
        int16_t             getADCValue();
        void                setBrother(aDSChannel *);

    protected:
        void                _turnOffPWM(aPin_t);
        void                _turnPWM(bool);
        int8_t              _digitalRead(aPin_t);
        void                _digitalWrite(aPin_t, uint8_t);
        uint16_t             _analogRead(aPin_t);
        void                _analogWrite(aPin_t, uint8_t);

    private:
        aPin_t                  m_pwmPin;
        aPin_t                  m_sensorPin;
        aPin_t                  m_ledPin;
        bool                    m_hasFocus;
        int16_t                 m_targetTemp;
        int16_t                 m_currentTemp;
        int8_t                  m_pwmValue;
        int16_t                 m_adcValue;
        bool                    m_inStandby;
        HeatingState_t          m_heatState;
        uint8_t                 m_ledState;
        unsigned long           m_nextPass;
        bool                    m_tempHasChanged;
        unsigned long           m_nextBlink;
        uint8_t                 m_blinkStandby;
        CalibrationData_t       m_cal;
#ifdef SIMU
        unsigned long           m_nextTempStep;
        unsigned long           m_nextLowering;
#endif // SIMU
#if CHANNEL_COUNTING
        uint8_t                 m_channel;
#endif
        bool                    m_isPlugged;
        aDSChannel             *m_brother;
        ValueAveraging          m_avrTemp;
};

class aDSEngine;

/// \brief aDSChannels class
///
class aDSChannels
{
    friend aDSEngine;

    public:
        static const uint8_t        OFFSET_VALUE                = 2;        ///< Value column LCD offset
        static const uint8_t        OFFSET_MARKER_LEFT          = 0;        ///< Column LCD offset for left marker '['
        static const uint8_t        OFFSET_MARKER_RIGHT         = 10;       ///< Column LCD offset for right marker ']'

        // Settings to Operation mode switch timeout
        static const unsigned long  OPERATION_SET_TIMEOUT       = 3000;     ///< Automatic toggle settings->reading timeout (3 seconds), in ms

        static const unsigned long  DISPLAY_UPDATE_RATE         = 200;      ///< Display update rate, in ms
        static const unsigned long  MEASURE_UPDATE_RATE         = 200;      ///< Measurement (for aDSChannel) rate, in ms

        static const unsigned long  TEMP_SETTING_INACTIVITY     = 30000;    ///< Timeout in ms, after which the new target temperature will be stored in the EEPROM.

        /// \brief Channels enumeration
        ///
        ///
        ///
        typedef enum
        {
            CHANNEL_ONE,
            CHANNEL_TWO,
            CHANNEL_MAX
        } Channel_t;

        static const uint16_t       DATA_CHANNEL2_ENABLED       = 1;        ///< Bitfield: Channel 2 is enabled
        static const uint16_t       DATA_CHANNELS_JOINDED       = 1 << 1;   ///< Bitfield: Channel 1 & 2 are joinded
        static const uint16_t       DATA_OPERATION              = 1 << 2;   ///< Bitfield: Operation mode has changed
        static const uint16_t       DATA_CHANNEL1_TEMP_SET      = 1 << 3;   ///< Bitfield: Target temperature of channel 1 has changed
        static const uint16_t       DATA_CHANNEL1_TEMP_READ     = 1 << 4;   ///< Bitfield: Readed temperature of channel 1 has changed
        static const uint16_t       DATA_CHANNEL1_LED_STATE     = 1 << 5;   ///< Bitfield: LED state of channel 1 has changed
        static const uint16_t       DATA_CHANNEL2_TEMP_SET      = 1 << 6;   ///< Bitfield: Target temperature of channel 1 has changed
        static const uint16_t       DATA_CHANNEL2_TEMP_READ     = 1 << 7;   ///< Bitfield: Readed temperature of channel 1 has changed
        static const uint16_t       DATA_CHANNEL2_LED_STATE     = 1 << 8;   ///< Bitfield: LED state of channel 1 has changed
        static const uint16_t       DATA_DISPLAY                = 1 << 9;   ///< Bitfield: Display should be refreshed
        static const uint16_t       DATA_STANDBY                = 1 << 10;  ///< Bitfield: Standby state
        static const uint16_t       DATA_DISPLAY_STANDBY        = 1 << 11;  ///< Bitfield: Standby state has changed
        static const uint16_t       DATA_FOCUS                  = 1 << 12;  ///< Bitfield: Focus has changed
        static const uint16_t       DATA_IN_CALIBRATION         = 1 << 13;  ///< Bitfield: in Calibration

    private:
        /** \brief Union to manipulate float/uint8_t [] calibration values
         *
         */
        union _eepromCalibrationValue_t
        {
            float v;
            uint8_t c[sizeof(float)];
        };

        static const int16_t        EEPROM_ADDR_MAGIC              = 0;       ///< EEPROM offset storage start for magic numbers (0xDEAD)
        static const int16_t        EEPROM_STORAGE_STARTING        = 5;       ///< EEPROM starting address for program datas
        static const int16_t        EEPROM_TEMP_SIZE               = sizeof(uint16_t) + sizeof(uint8_t);  ///< EEPROM temperature size (temperature + crc)
        static const int16_t        EEPROM_ADDR_CHANNEL_JOINED     = EEPROM_STORAGE_STARTING + 1;         ///< Channels are joinded
        static const int16_t        EEPROM_ADDR_TEMP_CHANNEL_ONE   = EEPROM_ADDR_CHANNEL_JOINED + EEPROM_TEMP_SIZE;    ///< Target temp for Channel 1
        static const int16_t        EEPROM_ADDR_TEMP_CHANNEL_TWO   = EEPROM_ADDR_TEMP_CHANNEL_ONE + EEPROM_TEMP_SIZE;  ///< Target temp for Channel 2

        static const int16_t        EEPROM_CALIBRATION_SIZE        = (sizeof(float) * 2) + sizeof(uint8_t);            ///< EEPROM calibration size: 2 float (slope & offset), and one uint8_t for crc
        static const int16_t        EEPROM_ADDR_CALIBRATION_CHAN_1 = EEPROM_ADDR_TEMP_CHANNEL_TWO + EEPROM_TEMP_SIZE;  ///< EEPROM start offset for Channel 1 calibration values
        static const int16_t        EEPROM_ADDR_CALIBRATION_CHAN_2 = EEPROM_ADDR_CALIBRATION_CHAN_1 + EEPROM_CALIBRATION_SIZE; ///< EEPROM start offset for Channel 2 calibration values

    public:
        aDSChannels(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
        virtual ~aDSChannels();

        void                setup(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
        void                setOperationMode(OperationMode_t);
        OperationMode_t     getOperationMode();
        void                updateOperationMode();
        void                pingOperationMode();

        #define             IS_DATA_ENABLED(bit) ((m_datas & bit))
        //bool                isDataEnabled(uint16_t);

        void                syncData(uint16_t);
        void                incEncoderPosition(uint16_t);
        void                service();
        void                toggleJoined();
        bool                isJoinded();
        void                setFocusToNextChannel();
        void                toggleStandbyMode();
        bool                isInStandby();

        void                setCalibrationValues(Channel_t, aDSChannel::CalibrationData_t);
        aDSChannel::CalibrationData_t getCalibrationValues(Channel_t);
        void                restoreCalibationValues();
        void                saveCalibrationValues(Channel_t);
        bool                isInCalibration();
        void                setCalibrationMode(bool);

    protected:
        void                _enableData(uint16_t, bool);
        void                _enableDataCheck(uint16_t, bool);
        void                _updateDisplay();
        void                _displayBigDigit(uint8_t, uint8_t, uint8_t = 0);
        void                _displayBigDigits(int16_t, uint8_t, uint8_t = 0);
        void                _clearValue(uint8_t, int = 0);
        void                _updateField(OperationMode_t, int16_t, uint8_t);
        void                _wakeupFromStandby();
        void                _showBanner();
        // EEPROM related functions
        bool                _checkForMagicNumbers();
        void                _writeMagicNumbers();
        uint8_t             _crc8(const uint8_t *, uint8_t);
        template <typename T>
        bool                _write(T const, int16_t &);
        template <typename T>
        bool                _read(T &, int16_t &);
        template <typename T>
        void                _scissor(T v, uint8_t *, size_t &);
        bool                _getTempFromEEPROM(int16_t, uint16_t &);
        void                _setTempToEEPROM(int16_t, uint16_t);
        void                _restoreCalibrationFromEEPROM(int16_t, aDSChannel &);
        void                _backupCalibrationFromEEPROM(int16_t, aDSChannel &);


    private:
        LiquidCrystal           m_lcd;
        aDSChannel              m_channels[CHANNEL_MAX];
        OperationMode_t         m_operationMode;
        unsigned long           m_operationTick;
        uint16_t                m_datas;
        uint8_t                 m_lcdCols;
        uint8_t                 m_lcdRows;
        unsigned long           m_nextDisplayUpdate;
        unsigned long           m_nextMeasureUpdate;
        unsigned long           m_lastTempChange;
        bool                    m_isValidEEPROM;
        bool                    m_storedToEEPROM;
};


/// \brief aDSEngine class
///
class aDSEngine
{
    private:
        static const uint8_t RXBUFFER_MAXLEN = 64; ///< USB input communication buffer's max size

    public:
        aDSEngine();
        virtual ~aDSEngine();

        void                setup();
        void                run();

    protected:
        void                _handleSerialInput();

    private:
        aDSChannels             m_channels;
        ClickEncoder            m_encoder;
        uint16_t                m_datas;
        uint8_t                 m_RXbuffer[RXBUFFER_MAXLEN];    ///< USB rx buffer
        uint8_t                 m_RXoffset;                     ///< USB rx buffer offset counter
        unsigned long           m_serialInputTick;
};

#endif // ADSENGINE_H
