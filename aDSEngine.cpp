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
#include "aDSEngine.h"

#include <wiring_private.h>

///
/// \file aDSEngine.cpp
/// \author F1RMB, Daniel Caujolle-Bert <f1rmb.daniel@gmail.com>
///

///
/// \page mainpage SMD Soldering Station for Weller RT Series Tips
///
///     Based on a project by __Martin Kumm__ http://www.martin-kumm.de/wiki/doku.php?id=Projects:SMD_Solderstation.
///
///  The hardware has been redesigned and modified (two channels, 16x2 LCD instead of 7 segments display, etc). The software has also been rewritten from scratch.
///
/// <br>
///  I would especially like to thank my friend __Olivier__, <i><b>F5LGJ</b></i>, for his great help and support in this project.
///
///  Big thumbs up to __Patrick__, <i><b>F6AZZ</b></i>, for the documentation corrections ;-)
///
///


///
/// \page UI User Interface overview
///
/// - The Soldering Station control is performed using a simple rotary encoder, which integrates a push button.
///
/// <br>
/// - The temperature range varies from 100°C, up to 450°C.
///
///
/// <br>
/// - Depending on the hardware assembly, the station can handle one or two soldering irons:
///
///     -# The Single mode, the temperature readings and settings are displayed using a double height font.
///
///     -# The dual channels version, both soldering irons can be controlled separately, or can be merged:
///         + when the separate channels mode is used, each channel is independent. Simple clicking on the encoder push button
///            will set the focus to the next channel. The focused channel's temperature will be surrounded by the symbols <b>[</b> and <b>]</b>
///
///         + when merged mode is used, the temperature is displayed in the same way as the <i>Single</i> channel mode (double height font), both
///            channels share the same settings (target temperature).
///
/// <br>
/// - LED status decoding:
///      LED Status | Meaning
///     ------------|---------
///     ON | the tip is heating
///     OFF | the tip is cooling
///     Blinking | the tip has reached his target temperature
///     Three times blinking | the soldering station is in Standby mode (see \ref standby)
///
///
/// <br>
/// - The target temperature is stored, for each channel, into the microcontroller's EEPROM.
///   The values will be restored on the next startup.
///   After a timeout of 30 seconds, a new defined target temperature will be stored into the EEPROM.
///   If in the meantime the user defines a new target temperature, the timeout will be reset.
///
///
/// <br>
/// - When the station is in the temperature reading mode, the displayed values are left aligned.
///   When the station is in the settings mode, the displayed values are right aligned.
///
///
///
/// <br>
/// \section encoderuse Encoder use
///
/// - In any mode (settings or temperature readings), the rotary encoder is used to define the target temperature. Turn the encoder clockwise to increase the
///   target temperature, and anti-clockwise to decrease it.
///
/// <br>
/// - When the soldering station is not in the settings mode, it displays the soldering tips temperature. A single encoder detents rotation
///   will switch the soldering station into the settings mode, and displays the target temperature without any change to the target temperature
///   settings.
///
/// <br>
/// - When the soldering station is in the settings mode, and no action is performed using the encoder rotation within 3 seconds, it will
///   switch back to the temperature readings.
///
/// <br>
/// - Encoder push button:
///
///     + Single soldering tip version:
///         Button | Action
///         -------|-------
///         Single Click | <i>no effect</i>
///         Double Click | switch to standby mode (see \ref standby)
///         Held | <i>no effect</i>
///
/// <br>
///     + Dual soldering tip version:
///         Button | Action
///         -------|-------
///         Single click | change the focus to the next channel (if not in merged mode)
///         Double click | switch to standby mode (see \ref standby)
///         Held | toggles mergded mode (see \ref merged)
///
///
/// <br>
/// \section merged Merged mode
///
/// - With Dual Channel enabled hardware, it is possible to share the same temperature preset values for both soldering tips.
///
/// See \ref encoderuse
///
///
/// <br>
/// \section standby Standby
///
/// - A double-click on the encoder brings the soldering station in the standby mode.
///
/// <br>
/// - When the standby mode is enabled, the target temperature will decrease to 150°C if the temperature setting is set
///   above this point, otherwise it will decrease to 100°C.
///
/// <br>
/// - Any encoder action will exits from <i>Standby</i> mode.
///
/// <br>
/// - When the <i>Standby</i> mode is activated, the LEDs blink three times cyclically.
///
///


///
/// \page cal Calibration Process
///
///
/// + __Prerequisites__:<br><br>
///
///      - __Hardware__:
///        - Digital Thermometer (e.g: your digital multimeter with a thermocouple)
/// <br><br>
///      - __Software__:
///         * A serial terminal emulator (e.g. “<i>HyperTerminal</i>” or “<i>Tera Term</i>” on Windows, “<i>minicom</i>” or “<i>cutecom</i>” on Linux).
///         * The calibration spreadsheet file <b>aWXIronsCalibration.ods</b>
///         * A software able to open the calibration spreadsheet, like “<i>LibreOffice</i>“, “<i>OpenOffice</i>“ and so on.
///
///         The serial communication settings are: <b>57600</b>, <b>8</b>, <b>N</b>, <b>1</b>
///
/// <br>
/// + __Why a calibration__:<br>
///
///     The calibration process is necessary get accurate temperature control.<br>
///     Some average values are used by default, but that won't gives you accurate temperature control.
///
/// <br>
/// + __Process Description__:<br><br>
///
///     - __Step 1: <i>Calibration Mode</i>__
///
///         You have to open the soldering station's box and connect the soldering station to the PC, using a USB cable.<br>
///         To turn the soldering station in calibration, you have to keep the encoder's push button pressed while turning ON the station.
///         Once the station is ready to use, the '<b><i>CAL</i></b>' string is displayed on the top left side of the LCD display.<br>
///
///         In calibration mode, the readed temperature isn't displayed anymore. Instead, the ADC value is shown.<br>
///
///         The station will self set the target temperature to 100°C. You have to wait until the temperature stabilizes.<br><br>
///
///     - __Step 2: <i>Calibrate Channel 1 or 2</i>__
///
///         Select matching <b><i>Channel</i></b> tab in the calibration spreadsheet file.<br>
///         Set the temperature target using the pseudo temperature value in the first column. Wait until LED blinks and the displayed ADC value stabilize.<br>
///         Apply the thermocouple to the soldering tip, then write down the readed temperature to the column named '<b><i>Temp °C'</i></b>.<br>
///         If needed, adjust the value in the '<b><i>ADCread</i></b>' column, accordingly to the one displayed on the station's LCD.<br>
///         Apply the same procedure for all spreadsheet's rows.<br>
///
///         Once you completed the array, down the chart, the '<i>Calibration String</i>' cell contains the string you have to copy and paste to the serial terminal emulator, e.g:
///         \code :CAL:1:0.3757498594,51.7808993467 \endcode
///         This string starts with '<b>:CAL:</b>', followed by the channel's name, then two floating point values, comma separated.<br>
///         Once the string entered and validated with the <b>[RETURN]</b> key, you should get a '<b><i>:OK:</i></b>' acknowledge message.<br>
///         In case you get '<b><i>:ERR:</i></b>', double check the calibration string you pasted.<br>
///
///         If you own a Dual Channel soldering station, <b>repeat this step for the second Channel</b>.<br><br>
///
///     - __Last Step: <i>Backup</i>__
///
///          Once the full calibration is done, you <b>HAVE</b> to store the new values into the EEPROM, using the following command:
///          \code :CAL:SAVE \endcode
///          As usual, you should get a '<i><b>:OK:</b><i>' acknoledge message.
///
///          Once you validate the calibration with this command, you leave the calibration mode.<br>
///          You can unplug the USB cable, close the box and use your soldering station now.
///
/// <br>
/// + __Other available calibration commands__:<br><br>
///
///     - <i>:CAL:</i><b>OFF</b>  Cancels the calibration process, restoring previous values.
///
///     - <i>:CAL:</i><b>DUMP</b>  Displays the calibration values (stored in EEPROM and current ones) in the serial terminal emulator
///
/// \warning If you own a Dual Channel soldering station, you <b>HAVE</b> to calibrate both channels, or at least enter the <i>old</i> calibration string for the channel you won't calibrate.
///



static const uint8_t        DIGIT_WIDTH          = 3;   ///< Max numerical length of temperature (used with big digits)
static const uint8_t        _glyphs[][8] PROGMEM =      ///< LCD glyphs (for big digits and LED)
{
    {
        B11111,
        B11111,
        B00000,
        B00000,
        B00000,
        B00000,
        B00000,
        B00000
    },
    {
        B00000,
        B00000,
        B00000,
        B00000,
        B00000,
        B00000,
        B11111,
        B11111
    },
    {
        B11111,
        B11111,
        B00000,
        B00000,
        B00000,
        B00000,
        B11111,
        B11111
    },
    {
        B11111,
        B11111,
        B11111,
        B11111,
        B11111,
        B11111,
        B11111,
        B11111
    },
    {
        B00000,
        B00000,
        B00000,
        B00000,
        B00000,
        B01110,
        B01110,
        B01110
    },
    {
        B00000,
        B00000,
        B00011,
        B00011,
        B00011,
        B00011,
        B00000,
        B00000
    },
    {
        B00000,
        B00000,
        B11000,
        B11000,
        B11000,
        B11000,
        B00000,
        B00000
    },
    { // LED
        B00000,
        B01110,
        B10011,
        B10111,
        B10111,
        B11111,
        B01110,
        B00000
    }
};

/// _glyphs[] offsets
static const uint8_t        _bigDigitsTop[12][DIGIT_WIDTH] =        ///< 0..9 + ' ' top characters matrix
{
    {  3,  0,  3 }, // 0
    {  0,  3, 32 }, // 1
    {  2,  2,  3 }, // 2
    {  0,  2,  3 }, // 3
    {  3,  1,  3 }, // 4
    {  3,  2,  2 }, // 5
    {  3,  2,  2 }, // 6
    {  0,  0,  3 }, // 7
    {  3,  2,  3 }, // 8
    {  3,  2,  3 }, // 9
    { 32, 32, 32 }, // empty
    {  1,  1,  1 }  // -
};

/// _glyphs[] offsets
static const uint8_t        _bigDigitsBottom[12][DIGIT_WIDTH] =     ///< 0..9 + ' ' bottom characters matrix
{
    {  3,  1,  3 }, // 0
    {  1,  3,  1 }, // 1
    {  3,  1,  1 }, // 2
    {  1,  1,  3 }, // 3
    { 32, 32,  3 }, // 4
    {  1,  1,  3 }, // 5
    {  3,  1,  3 }, // 6
    { 32, 32,  3 }, // 7
    {  3,  1,  3 }, // 8
    {  1,  1,  3 }, // 9
    { 32, 32, 32 }, // empty
    {  0,  0,  0 }  // -
};

static ClickEncoder *pEncoder = NULL;  ///< Global pointer to ClickEncoder object, used inside timer1ISR() function

#if CHANNEL_COUNTING
static uint8_t channelCount = 0;
#endif


/// \brief Returns numerical character length of argument
///
/// \param n int16_t : value to get length from
/// \return int8_t : length
///
///
static int8_t getNumericalLength(int16_t n)
{
    char buf[16];

    return (static_cast<int8_t>(snprintf_P(buf, sizeof(buf) - 1, PSTR("%d"), n)));
}

//
// Begin of Class aDSTemperatureAveraging
//
#ifdef DOUBLE_AVERAGE
/// \brief ValueAveraging class constructor
///
///
ValueAveraging::ValueAveraging() :
		m_average(ARRAY_SIZE_MAX)
{
    // ctor
	ResetValues();
}

/// \brief ValueAveraging class destructor
///
///
ValueAveraging::~ValueAveraging()
{
    // dtor
}

/// \brief Stacks the value to an array, used to compute an averaged value
///
/// \param value int16_t : value to stack
/// \return void
///
///
template<typename T>
void ValueAveraging::StackValue(T value)
{
    if (value > 0)
    {
        m_offset = (m_offset + 1) % m_average;

        m_values[m_offset] = static_cast<double>(value);
    }
}

/// \brief Returns the averaged value, computed from stacked values
///
/// \return int16_t : averaged value
///
///
template<typename T>
T ValueAveraging::GetValue()
{
    uint16_t n = 0;
    double	 sum = 0.0;

    for (uint16_t i = 0; i < m_average; i++)
    {
    	if (isnan(m_values[i]))
    		break;

        if (m_values[i] > 0)
        {
            sum += m_values[i];
            n++;
        }
    }

    // No usable value found.
    if (n == 0)
        return 0;

    return static_cast<T>((sum / double(n)) + 0.5); // ceil
}


/// \brief Set how many values will be used to compute the average
///
/// \param v uint16_t : values used for averaging
/// \return bool : true on success
///
///
bool ValueAveraging::SetAverage(uint16_t v)
{
	bool ret = false;

	if ((v >= 0) && (v <= ARRAY_SIZE_MAX))
	{
		// Zeroing the array
		ResetValues();

		m_average = v;

		ret = true;
	}

	return ret;
}

/// \brief Get how many values will be used to compute the average
///
/// \return uint16_t : values used for averaging
///
///
uint16_t ValueAveraging::GetAverage()
{
	return m_average;
}

/// \brief Get max value that can be used to build the average
///
/// \return uint16_t : max values used for averaging
///
///
uint16_t ValueAveraging::GetMaxAverage()
{
	return ARRAY_SIZE_MAX;
}

/// \brief Reset the array used to store values to be averaged
///
/// \return void
///
///
void ValueAveraging::ResetValues()
{

	for (uint16_t i = 0; i < ARRAY_SIZE_MAX; i++)
		m_values[i] = NAN;

	m_offset = ARRAY_SIZE_MAX - 1;
}
#else
/// \brief ValueAveraging class constructor
///
///
ValueAveraging::ValueAveraging() :
		m_average(ARRAY_SIZE_MAX)
{
    // ctor
	ResetValues();
}

/// \brief ValueAveraging class destructor
///
///
ValueAveraging::~ValueAveraging()
{
    // dtor
}

/// \brief Stacks the value to an array, used to compute an averaged value
///
/// \param value int16_t : value to stack
/// \return void
///
///
void ValueAveraging::StackValue(int16_t value)
{
    if (value > 0)
    {
        m_offset = (m_offset + 1) % m_average;

        m_values[m_offset] = value;
    }
}

/// \brief Returns the averaged value, computed from stacked values
///
/// \return int16_t : averaged value
///
///
int16_t ValueAveraging::GetValue()
{
    uint16_t n = 0;
    double	 sum = 0.0;

    for (uint16_t i = 0; i < m_average; i++)
    {
    	if (m_values[i] == -1)
    		break;

        if (m_values[i] > 0)
        {
            sum += m_values[i];
            n++;
        }
    }

    // No usable value found.
    if (n == 0)
        return 0;

    return static_cast<int16_t>((sum / double(n)) + 0.5); // ceil
}


/// \brief Set how many values will be used to compute the average
///
/// \param v uint16_t : values used for averaging
/// \return bool : true on success
///
///
bool ValueAveraging::SetAverage(uint16_t v)
{
	bool ret = false;

	if ((v >= 0) && (v <= ARRAY_SIZE_MAX))
	{
		// Zeroing the array
		ResetValues();

		m_average = v;

		ret = true;
	}

	return ret;
}

/// \brief Get how many values will be used to compute the average
///
/// \return uint16_t : values used for averaging
///
///
uint16_t ValueAveraging::GetAverage()
{
	return m_average;
}

/// \brief Get max value that can be used to build the average
///
/// \return uint16_t : max values used for averaging
///
///
uint16_t ValueAveraging::GetMaxAverage()
{
	return ARRAY_SIZE_MAX;
}

/// \brief Reset the array used to store values to be averaged
///
/// \return void
///
///
void ValueAveraging::ResetValues()
{

	for (uint16_t i = 0; i < ARRAY_SIZE_MAX; i++)
		m_values[i] = -1;

	m_offset = ARRAY_SIZE_MAX - 1;
}
#endif
//
// End of Class ValueAveraging
//

//
// Begin of Class aDSChannel
//
/// \brief aDSChannel class constructor
///
aDSChannel::aDSChannel() :
    m_hasFocus(false),
    m_targetTemp(0),
    m_currentTemp(0),
    m_pwmValue(0),
    m_adcValue(0),
    m_inStandby(false),
    m_heatState(HEATING_STATE_COOLING),
    m_ledState(LOW),
    m_nextPass(0),
    m_tempHasChanged(true),
    m_nextBlink(0),
    m_blinkStandby(0)
#ifdef SIMU
    , m_nextTempStep(0),
    m_nextLowering(0)
#endif // SIMU
#if CHANNEL_COUNTING
    , m_channel(channelCount++)
#endif // 0
    , m_isPlugged(true),
    m_brother(NULL)
{
    //ctor
    m_pwmPin.pin    = NOT_A_PIN;
    m_sensorPin.pin = NOT_A_PIN;
    m_ledPin.pin    = NOT_A_PIN;

    m_cal.slope     = DEFAULT_TEMPERATURE_SLOPE;
    m_cal.offset    = DEFAULT_TEMPERATURE_OFFSET;
}

/// \brief aDSChannel destructor
///
aDSChannel::~aDSChannel()
{
    //dtor
#if CHANNEL_COUNTING
    channelCount--;
#endif
}

/// \brief Setup member function, should be called before any other member
///
/// Pins will be embedded into aPin_t object, timer, mask, port and output register will be set also here,
/// preventing using Arduino analog/digital{Read/Write}() calls, which are quite slow.
///
/// \param pwmPin uint8_t : PWM pin, used to drive the output MosFET
/// \param sensorPin uint8_t : Sensor pin, used to get analog temperature value.
/// \param ledPin uint8_t : LED pin, used to reflect Heating/Cooling state
/// \return void
///
///
void aDSChannel::setup(uint8_t pwmPin, uint8_t sensorPin, uint8_t ledPin)
{
    //
    // PWM
    //
    m_pwmPin.pin            = pwmPin;
    pinMode(m_pwmPin.pin, OUTPUT);
    m_pwmPin.timer          = digitalPinToTimer(m_pwmPin.pin);
    m_pwmPin.mask           = digitalPinToBitMask(m_pwmPin.pin);
    m_pwmPin.port           = digitalPinToPort(m_pwmPin.pin);
    m_pwmPin.outputRegister = portOutputRegister(m_pwmPin.port);
    //
    // Turn PWM OFF
    //
    _analogWrite(m_pwmPin, m_pwmValue);

    //
    // Sensor
    //
    m_sensorPin.pin = sensorPin;
    pinMode(m_sensorPin.pin, INPUT),

    //
    // LED
    //
    m_ledPin.pin            = ledPin;
    pinMode(m_ledPin.pin, OUTPUT);
    m_ledPin.timer          = digitalPinToTimer(m_ledPin.pin);
    m_ledPin.mask           = digitalPinToBitMask(m_ledPin.pin);
    m_ledPin.port           = digitalPinToPort(m_ledPin.pin);
    m_ledPin.outputRegister = portOutputRegister(m_ledPin.port);
    //
    // Turn LED OFF
    //
    _digitalWrite(m_ledPin, HIGH);
}

/// \brief Set the focus, as display point of view.
///
/// \param v bool : focus state
/// \return void
///
///
void aDSChannel::setFocus(bool v)
{
    m_hasFocus = v;
}

/// \brief Get the focus state.
///
/// \return bool : focus state
///
///
bool aDSChannel::hasFocus()
{
    return m_hasFocus;
}

/// \brief Get current temperature accordingly from the given mode (SET/READ)
///
/// \param mode OperationMode_t : operation mode
/// \return uint16_t : temperature, in Celcius
///
///
uint16_t aDSChannel::getTemperature(OperationMode_t mode)
{
    return (mode == OPERATION_MODE_READ) ?

#ifdef DOUBLE_AVERAGE
    m_avrTemp.GetValue<int16_t>()
#else
    m_avrTemp.GetValue()
#endif // DOUBLE_AVERAGE


    : m_targetTemp;
}

/// \brief Set current temperature accordingly from the given mode (SET/READ)
///
/// \param mode OperationMode_t : operation mode
/// \param temp int16_t : temperature, in Celcius
/// \return bool : true if temperature has been changed, otherwise false
///
///
bool aDSChannel::setTemperature(OperationMode_t mode, int16_t temp)
{
    int16_t p = (mode == OPERATION_MODE_SET) ? m_targetTemp : m_currentTemp;

    if (mode == OPERATION_MODE_SET)
    {
        m_tempHasChanged = (m_targetTemp != temp); // Set bool flag if target temp has changed
        m_targetTemp = temp;
    }
    else
    {
        m_currentTemp = temp;
        m_avrTemp.StackValue(m_currentTemp);
    }

    return (p != ((mode == OPERATION_MODE_SET) ? m_targetTemp : m_currentTemp));
}

/// \brief This member should be called often, it manage heating/cooling of the Channel
///
/// \param m unsigned long : current timestamp
/// \return bool : return true if readed temperature has changed since last call, otherwise false
///
///
bool aDSChannel::service(unsigned long m)
{
    int16_t otemp = m_currentTemp;
    int16_t target = m_targetTemp;

    //
    // If temperature setting is < TEMPERATURE_STANDBY, then define it to TEMPERATURE_MIN insteaf of TEMPERATURE_STANDBY
    //
    if (m_inStandby)
        target = (m_targetTemp < TEMPERATURE_STANDBY) ? TEMPERATURE_MIN : TEMPERATURE_STANDBY;

    //
    // Get current temperature
    //
    // Turn OFF brother's heater, since that introduce some ADC noise
    //
    if (m_brother)
        m_brother->_turnPWM(false);

    //
    // Switch OFF the heater
    //
    _analogWrite(m_pwmPin, 0);

    //
    // Wait for some time (to get low pass filter in steady state)
    //
    delay(20);

    //
    // Read the ADC value
    //
    m_adcValue = _analogRead(m_sensorPin);

    //
    // Switch the heater back ON, to previous value
    //
    _analogWrite(m_pwmPin, m_pwmValue);

    //
    // Restore brother's heater state
    //
    if (m_brother)
        m_brother->_turnPWM(true);

    //
    // Detect if iron tip is plugged or not
    //
    m_isPlugged = (m_adcValue > 80);

    //
    // Compute temperature from ADC value
    //
    int16_t currTemp = round((static_cast<float>(m_adcValue) * m_cal.slope) + m_cal.offset);

#ifdef SIMU // DEV SIMULATION MODE
    currTemp = m_currentTemp;
#else
    m_currentTemp = currTemp;
    m_avrTemp.StackValue(m_currentTemp);
#endif

    //
    // Heating/Cooling
    //
    // Compute difference between target and actual temperature
    //
    int16_t diff = target - currTemp;

    int16_t pwm = 0;

    //
    // Limit PWM value to 0...PWM_MAX_VALUE
    //
    if (diff > 0)
    {
        pwm = constrain((diff * 10), 0, PWM_MAX_VALUE);

        //
        // Slightly increase PWM width for fine temp control
        //
        if (diff <= TEMPERATURE_TOLERANCE)
            pwm += (diff * 15);
    }

#if 0
    Serial.print(m_channel, DEC);
    Serial.print(F(": [ "));
    Serial.print(m_adcValue, DEC);
    Serial.print(F(" ], "));
    Serial.print(currTemp, DEC);
    Serial.print(F(" deg"));
    Serial.print(F(", diff: "));
    Serial.print(diff, DEC);
    Serial.print(F(", PWM: "));
    Serial.print(pwm, DEC);
    Serial.print(F(", "));
    Serial.println(m_isPlugged ? F("PLUGGED") : F("NOT PLUGGED"));
#endif

    //
    // Reflect Heating/Cooling/Reached on state LED
    //
    if (m_isPlugged)
    {
        if (m_inStandby)
            m_heatState = HEATING_STATE_STANDBY;
        else
        {
            if (currTemp > TEMPERATURE_TOLERANCE)
            {
                if ((currTemp >= (target - TEMPERATURE_TOLERANCE)) && (currTemp <= (target + TEMPERATURE_TOLERANCE)))
                    m_heatState = HEATING_STATE_REACHED;
                else
                    m_heatState = (pwm == 0) ? HEATING_STATE_COOLING : HEATING_STATE_HEATING;
            }
        }
    }
    else
        m_heatState = HEATING_STATE_COOLING;


#ifdef SIMU // DEV SIMULATION MODE
    //static unsigned long m_nextTempStep = 0;
    if ((m - m_nextTempStep) > 400)
    {
        m_nextTempStep = m;

        if (m_currentTemp > target)
            m_currentTemp--;
        else if (m_currentTemp < target)
            m_currentTemp++;

        if (((m_heatState == HEATING_STATE_REACHED) || (m_heatState == HEATING_STATE_STANDBY)) && ((m - m_nextLowering) > 1500))
        {
            m_nextLowering = m;
            m_currentTemp -= 2;
        }
    }
#endif

    //
    // Update PWM if necessary
    //
    if (m_pwmValue != pwm)
    {
        m_pwmValue = pwm;
        _analogWrite(m_pwmPin, m_pwmValue);
    }

    return (otemp != m_currentTemp);
}

/// \brief Enable or disable channel's standby
///
/// \param enable bool : enability
/// \return void
///
///
void aDSChannel::setStandbyMode(bool enable)
{
    m_inStandby = enable;
}

/// \brief Get channel's standby enability
///
/// \return bool
///
///
bool aDSChannel::getStandbyMode()
{
    return m_inStandby;
}

/// \brief Is target temperature has changed (use for EEPROM storage)
///
/// \return bool : return true if target temperature has changed, otherwise false
///
///
bool aDSChannel::isTempHasChanged()
{
    return m_tempHasChanged;
}

/// \brief Reset temperature change flag (use for EEPROM storage)
///
/// \return void
///
///
void aDSChannel::syncTempChange()
{
    m_tempHasChanged = false;
}

/// \brief Change LED state according for Heating/Cooling/Standby status
///
/// \param m unsigned long : timestamp
/// \return uint8_t : HIGH or LOW (LED on or off, accordingly)
///
///
uint8_t aDSChannel::updateLEDState(unsigned long m)
{
    if (((m - m_nextBlink) > BLINK_UPDATE_RATE) || (m_inStandby && ((m - m_nextBlink) > (BLINK_UPDATE_RATE / 3))))
    {
        m_nextBlink = m;

        switch (m_heatState)
        {
            case HEATING_STATE_HEATING:
                //
                // LED ON
                //
                if (_digitalRead(m_ledPin) == HIGH)
                    m_ledState = HIGH;
                break;

            case HEATING_STATE_COOLING:
                //
                // LED OFF
                //
                if (_digitalRead(m_ledPin) == LOW)
                    m_ledState = LOW;
                break;

            case HEATING_STATE_REACHED:
                //
                // BLINK
                //
                m_ledState = _digitalRead(m_ledPin);
                break;

            case HEATING_STATE_STANDBY:
                //
                // Triple BLINK
                //
                m_blinkStandby++;

                if (m_blinkStandby < 8)
                {
                    if (m_blinkStandby == 1)
                        m_ledState = LOW;
                    else
                        m_ledState = _digitalRead(m_ledPin);
                }
                else if (m_blinkStandby == 10)
                    m_blinkStandby = 0;
                break;
        }

        _digitalWrite(m_ledPin, !m_ledState);
    }

    return m_ledState;
}

/// \brief Get state LED status
///
/// \return uint8_t : HIGH or LOW (LED on or off, accordingly)
///
///
uint8_t aDSChannel::getLEDState()
{
    return m_ledState;
}

/// \brief Get channel heating state
///
/// \return aDSChannel::HeatingState_t : heating state
///
///
aDSChannel::HeatingState_t aDSChannel::getHeatState()
{
    return m_heatState;
}

/// \brief Get is tip plugged state
///
/// \return bool : plugged state
///
///
bool aDSChannel::isPlugged()
{
    return m_isPlugged;
}

/// \brief Set calibration data values
///
/// \param slope float : slope value
/// \param offset float : offset value
/// \return void
///
///
void aDSChannel::setCalibration(float slope, float offset)
{
    m_cal.slope = slope;
    m_cal.offset = offset;
}

/// \brief Get calibration data values
///
/// \return const aDSChannel::CalibrationData_t : Calibration data
///
///
const aDSChannel::CalibrationData_t aDSChannel::getCalibration() const
{
    return m_cal;
}

/// \brief Get latest ADC value
///
/// \return int16_t : ADC value
///
///
int16_t aDSChannel::getADCValue()
{
    return m_adcValue;
}

/// \brief Set relationship
///
/// \param p aDSChannel* : pointer to other aDSChannel
/// \return void
///
///
void aDSChannel::setBrother(aDSChannel *p)
{
    m_brother = p;
}

/// \brief Turns PWM off for the given pin
///
/// Took and Hacked from Arduino wiring_digital.c
///
/// \param pin aPin_t : pin
/// \return void
///
///
void aDSChannel::_turnOffPWM(aPin_t pin)
{
        switch (pin.timer)
        {
            case TIMER0A:
                cbi(TCCR0A, COM0A1);
                break;

            case TIMER0B:
                cbi(TCCR0A, COM0B1);
                break;

#if 0 // Unused
            case TIMER1A:
                cbi(TCCR1A, COM1A1);
                break;

            case TIMER1B:
                cbi(TCCR1A, COM1B1);
                break;

            case TIMER2A:
                cbi(TCCR2A, COM2A1);
                break;

            case TIMER2B:
                cbi(TCCR2A, COM2B1);
                break;
#endif
        }
}

void aDSChannel::_turnPWM(bool enable)
{
    if (enable)
        _analogWrite(m_pwmPin, m_pwmValue);
    else
        _analogWrite(m_pwmPin, 0);
}

/// \brief Get state of the given pin
///
/// Took and Hacked from Arduino wiring_digital.c
///
/// \param pin aPin_t : pin
/// \return int8_t : HIGH or LOW
///
///
int8_t aDSChannel::_digitalRead(aPin_t pin)
{
	if (pin.port == NOT_A_PIN)
        return LOW;

    //
	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	//
	if (pin.timer != NOT_ON_TIMER)
        _turnOffPWM(pin);

	if (*(pin.outputRegister) & (pin.mask))
        return HIGH;

	return LOW;
}

/// \brief Set state of the given pin
///
/// Took and Hacked from Arduino wiring_digital.c
///
/// \param pin aPin_t : pin
/// \param val uint8_t : state (HIGH or LOW)
/// \return void
///
///
void aDSChannel::_digitalWrite(aPin_t pin, uint8_t val)
{
    //
    // If the pin that support PWM output, we need to turn it off
    // before doing a digital write.
    //
    if (pin.timer != NOT_ON_TIMER)
        _turnOffPWM(pin);

    uint8_t oldSREG = SREG;

    cli();

    if (val == LOW)
        *(pin.outputRegister) &= ~(pin.mask);
    else
        *(pin.outputRegister) |= (pin.mask);

    SREG = oldSREG;
}

/// \brief Get analog value of the given pin
///
/// Took and Hacked from Arduino wiring_analog.c
///
/// \param pin aPin_t : pin
/// \return uint8_t : analog value (0..255)
///
///
uint16_t aDSChannel::_analogRead(aPin_t pin)
{
    uint8_t low, high;
    uint8_t _pin = pin.pin;

	if (_pin >= 14)
        _pin -= 14; // allow for channel or pin numbers

    ADMUX = (DEFAULT << 6) | (_pin & 0x07);

    //
    // start the conversion
    //
    sbi(ADCSRA, ADSC);

    //
    // ADSC is cleared when the conversion finishes
    //
    while (bit_is_set(ADCSRA, ADSC));

    //
    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    //
    low  = ADCL;
    high = ADCH;

    //
    // combine the two bytes
    //
    return (high << 8) | low;
}

/// \brief Set analog value for the given pin
///
/// Took and Hacked from Arduino wiring_analog.c
///
/// \param pin aPin_t : pin
/// \param val uint8_t : analog value (0..255)
/// \return void
///
///
void aDSChannel::_analogWrite(aPin_t pin, uint8_t val)
{
    //
    // We need to make sure the PWM output is enabled for those pins
    // that support it, as we turn it off when digitally reading or
    // writing with them.  Also, make sure the pin is in output mode
    // for consistenty with Wiring, which doesn't require a pinMode
    // call for the analog output pins.
    //

    if (val == 0)
        _digitalWrite(pin, LOW);
    else if (val == 255)
        _digitalWrite(pin, HIGH);
    else
    {
        switch(pin.timer)
        {
            case TIMER0A:
                //
                // connect pwm to pin on timer 0, channel A
                //
                sbi(TCCR0A, COM0A1);
                OCR0A = val; // set pwm duty
                break;

            case TIMER0B:
                //
                // connect pwm to pin on timer 0, channel B
                //
                sbi(TCCR0A, COM0B1);
                OCR0B = val; // set pwm duty
                break;

#if 0 // Unused
			case TIMER1A:
			    //
				// connect pwm to pin on timer 1, channel A
				//
				sbi(TCCR1A, COM1A1);
				OCR1A = val; // set pwm duty
				break;

			case TIMER1B:
			    //
				// connect pwm to pin on timer 1, channel B
				//
				sbi(TCCR1A, COM1B1);
				OCR1B = val; // set pwm duty
				break;

			case TIMER2A:
			    //
				// connect pwm to pin on timer 2, channel A
				//
				sbi(TCCR2A, COM2A1);
				OCR2A = val; // set pwm duty
				break;

			case TIMER2B:
			    //
				// connect pwm to pin on timer 2, channel B
				//
				sbi(TCCR2A, COM2B1);
				OCR2B = val; // set pwm duty
				break;

			case NOT_ON_TIMER:
			default:
				if (val < 128)
					_digitalWrite(pin, LOW);
				else
					_digitalWrite(pin, HIGH);
#endif
        }
    }
}
//
// End of Class aDSChannel
//

//
// Begin of Class aDSChannels
//
/// \brief aDSChannels constructor
///
/// \param rs uint8_t : LCD RS pin
/// \param e uint8_t : LCD Enable pin
/// \param d4 uint8_t : LCD D4 pin
/// \param d5 uint8_t : LCD D5 pin
/// \param d6 uint8_t : LCD D6 pin
/// \param d7 uint8_t : LCD D7 pin
///
///
aDSChannels::aDSChannels(uint8_t rs, uint8_t e, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) :
    m_lcd(rs, e, d4, d5, d6, d7),
    m_operationMode(OPERATION_MODE_SET),
    m_operationTick(0),
    m_datas(0x0),
    m_lcdCols(0),
    m_lcdRows(0),
    m_nextDisplayUpdate(0),
    m_nextMeasureUpdate(0),
    m_lastTempChange(0),
    m_isValidEEPROM(false),
    m_storedToEEPROM(false)
{
    //ctor
    _enableData(DATA_DISPLAY, true); // Full redraw on startup
}

/// \brief aDSChannels destructor
///
aDSChannels::~aDSChannels()
{
    //dtor
}

/// \brief Display a banner on the LCD
///
/// \return void
///
///
void aDSChannels::_showBanner()
{
    char buf[LCD_COLS + 1];

    m_lcd.clear();

    //
    // Reflect Dual or Single running version
    //
    snprintf_P(buf, sizeof(buf), PSTR("%s %s"), "aWXIrons", (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) ? "Dual" : "Single"));
    m_lcd.setCursor((LCD_COLS - strlen(buf)) / 2, 0);
    m_lcd.print(buf);

    //
    // Program's version
    //
    snprintf_P(buf, sizeof(buf), PSTR("%c %d.%d"), 'v', PROGRAM_VERSION_MAJOR, PROGRAM_VERSION_MINOR);
    m_lcd.setCursor((LCD_COLS - strlen(buf)) / 2, 1);
    m_lcd.print(buf);

    //
    // Wait 1s
    //
    delay(1000);
    m_lcd.clear();

    //
    // FOSS - FOSH licenses
    //
    snprintf_P(buf, sizeof(buf), PSTR("%s"), "FOSS  -  FOSH");
    m_lcd.setCursor((LCD_COLS - strlen(buf)) / 2, 0);
    m_lcd.print(buf);

    //
    // Author
    //
    snprintf_P(buf, sizeof(buf), PSTR("%s  2015-%d"), "F1RMB", PROGRAM_YEAR);
    m_lcd.setCursor((LCD_COLS - strlen(buf)) / 2, 1);
    m_lcd.print(buf);

    //
    // Wait 1s
    //
    delay(1000);
    m_lcd.clear();
}

/// \brief Check for the magic number in the EEPROM
///
/// \return bool : true if magic numbers has been found, otherwise false
///
///
bool aDSChannels::_checkForMagicNumbers()
{
    if (!m_isValidEEPROM)
        m_isValidEEPROM = ((EEPROM.read(EEPROM_ADDR_MAGIC) == 0xD) && (EEPROM.read(EEPROM_ADDR_MAGIC + 1) == 0xE) && (EEPROM.read(EEPROM_ADDR_MAGIC + 2) == 0xA) && (EEPROM.read(EEPROM_ADDR_MAGIC + 3) == 0xD));

    return m_isValidEEPROM;
}

/// \brief Write magic numbers into EEPROM
///
/// \return void
///
///
void aDSChannels::_writeMagicNumbers()
{
    //
    // Magic numbers
    //
    EEPROM.write(EEPROM_ADDR_MAGIC,     0xD);
    EEPROM.write(EEPROM_ADDR_MAGIC + 1, 0xE);
    EEPROM.write(EEPROM_ADDR_MAGIC + 2, 0xA);
    EEPROM.write(EEPROM_ADDR_MAGIC + 3, 0xD);
}

/// \brief CRC8 computation
///
/// Code took from http://www.pjrc.com/teensy/td_libs_OneWire.html
///
/// \param addr const uint8_t* : <b> Data source </b>
/// \param len uint8_t : <b> Data source length </b>
/// \return uint8_t : <b> CRC </b>
///
///
uint8_t aDSChannels::_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--)
    {
		uint8_t inbyte = *addr++;

		for (uint8_t i = 8; i; i--)
        {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;

			if (mix)
                crc ^= 0x8C;

			inbyte >>= 1;
		}
	}

	return crc;
}

/// \brief Template to write a value into EEPROM, at given address
///
/// \param v T const : value
/// \param addr int16_t& : start address
/// \return template \<typename T\> bool : true on write success, otherwise false
///
///
template <typename T>
bool aDSChannels::_write(T const v, int16_t &addr)
{
    if (!_checkForMagicNumbers())
        return false;

    union
    {
        T       v;
        uint8_t c[sizeof(T)];
    } value;

    value.v = v;

    for (size_t d = 0; d < sizeof(T); d++)
        EEPROM.write(addr++, value.c[d]);

    return true;
}

/// \brief Template to read a value from the EEPROM, at given address
///
/// \param v T& : readed value
/// \param addr int16_t& : start address
/// \return template \<typename T\> bool : true on read success, otherwise false
///
///
template <typename T>
bool aDSChannels::_read(T &v, int16_t &addr)
{
    union
    {
        T       v;
        uint8_t c[sizeof(T)];
    } value;

    v = 0;

    if (!_checkForMagicNumbers())
        return false;

    for (size_t d = 0; d < sizeof(T); d++)
        value.c[d] = EEPROM.read(addr++);

    v = value.v;

    return true;
}

/// \brief Template to decompose the value into an array of uint8_t (used for CRC8 computation)
///
/// \param v T : value
/// \param dest uint8_t* : pointer to the destination array
/// \param offset size_t& : start offset of the array
/// \return template \<typename T\> void
///
///
template <typename T>
void aDSChannels::_scissor(T v, uint8_t *dest, size_t &offset)
{
    union
    {
        T       v;
        uint8_t c[sizeof(T)];
    } value;

    value.v = v;

    for (size_t d = 0; d < sizeof(T); d++)
        dest[offset++] = value.c[d];
}

/// \brief Helper to read the stored temperature inside EEPROM at given address
///
/// \param startAddr int16_t : start address
/// \param temp uint16_t& : temperature
/// \return bool : true if the CRCs matches
///
///
bool aDSChannels::_getTempFromEEPROM(int16_t startAddr, uint16_t &temp)
{
    uint8_t  crcData[EEPROM_TEMP_SIZE - 1];
    size_t   crcOffset = 0;
    int16_t  start = startAddr;
    uint8_t  crc;

    //
    // Temperature
    //
    _read(temp, start);
    //
    // CRC8
    //
    _read(crc, start);
    //
    // Split
    //
    _scissor(temp, &crcData[0], crcOffset);

    return (_crc8(crcData, crcOffset) == crc);
}

/// \brief Helper to write a temperature inside EEPROM at given address
///
/// \param startAddr int16_t : start address
/// \param temp uint16_t : temperature
/// \return void
///
///
void aDSChannels::_setTempToEEPROM(int16_t startAddr, uint16_t temp)
{
    int16_t start = startAddr;
    uint8_t crcData[EEPROM_TEMP_SIZE - 1];
    size_t  crcOffset = 0;

    //
    // Split temp to uint8_t array
    //
    _scissor(temp, &crcData[0], crcOffset);
    //
    // Write temp
    //
    _write(temp, start);
    //
    // Write computed CRC8
    //
    _write(_crc8(crcData, crcOffset), start);
}

/// \brief Restore calibation value for given channel
///
/// \param startAddr int16_t : EEPROM start address
/// \param channel aDSChannel& : channel
/// \return void
///
///
void aDSChannels::_restoreCalibrationFromEEPROM(int16_t startAddr, aDSChannel &channel)
{
    int16_t                     start = startAddr;
    _eepromCalibrationValue_t   sl, off;
    uint8_t                     crc;
    uint8_t                     crcData[sizeof(float) * 2];
    uint8_t                     crcOffset = 0;

    for (uint8_t i = 0; i < sizeof(float); i++)
    {
        sl.c[i] = EEPROM.read(start++);
        crcData[crcOffset++] = sl.c[i];
    }

    for (uint8_t i = 0; i < sizeof(float); i++)
    {
        off.c[i] = EEPROM.read(start++);
        crcData[crcOffset++] = off.c[i];
    }

    crc = EEPROM.read(start++);

    if (crc == _crc8(crcData, crcOffset))
        channel.setCalibration(sl.v, off.v);
}

/// \brief Backup calibration value for given channel
///
/// \param startAddr int16_t : EEPROM start address
/// \param channel aDSChannel& : channel
/// \return void
///
///
void aDSChannels::_backupCalibrationFromEEPROM(int16_t startAddr, aDSChannel &channel)
{
    aDSChannel::CalibrationData_t   cal = channel.getCalibration();
    int16_t                         start = startAddr;
    _eepromCalibrationValue_t       sl, off;
    uint8_t                         crcData[sizeof(float) * 2];
    uint8_t                         crcOffset = 0;

    sl.v = cal.slope;
    off.v = cal.offset;

    if (!_checkForMagicNumbers())
        return;

    for (uint8_t i = 0; i < sizeof(float); i++)
    {
        EEPROM.write(start++, sl.c[i]);
        crcData[crcOffset++] = sl.c[i];
    }

    for (uint8_t i = 0; i < sizeof(float); i++)
    {
        EEPROM.write(start++, off.c[i]);
        crcData[crcOffset++] = off.c[i];
    }

    EEPROM.write(start++, (_crc8(crcData, crcOffset)));
}


/// \brief Setup member function, should be called before any other member
///
/// \param cols uint8_t : LCD number of columns
/// \param rows uint8_t : LCD number of rows
/// \param pwmChan1 uint8_t : Channel 1 PWM pin
/// \param sensChan1 uint8_t : Channel 1 Temperature Sensor pin
/// \param ledChan1 uint8_t : Channel 1 LED pin
/// \param chkChan2 uint8_t : Channel 2 enability pin
/// \param pwmChan2 uint8_t : Channel 2 PWM pin
/// \param sensChan2 uint8_t : Channel 2 Temperature Sensor pin
/// \param ledChan2 uint8_t : Channel 2 LED pin
/// \return void
///
///
void aDSChannels::setup(uint8_t cols, uint8_t rows,
                        uint8_t pwmChan1, uint8_t sensChan1, uint8_t ledChan1,
                        uint8_t chkChan2,
                        uint8_t pwmChan2, uint8_t sensChan2, uint8_t ledChan2)
{
    bool     joined = false;
    uint16_t temp;

    m_lcdCols = cols;
    m_lcdRows = rows;

    m_lcd.begin(cols, rows);

    //
    // Create custom glyphs
    //
    for(uint8_t i = 0; i < sizeof(_glyphs) / sizeof(_glyphs[0]); i++)
    {
        uint8_t gl[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

        for (uint8_t j = 0 ; j < 8; j++)
            gl[j] = pgm_read_byte(&_glyphs[i][j]);

        m_lcd.createChar(i, gl);
    }

    //
    // PWM
    //
    // Set PWM prescale factor to 1024
    //
    TCCR0B = _BV(CS00) | _BV(CS02);

    //
    // EEPROM storage checking
    //
    if (!_checkForMagicNumbers())
        _writeMagicNumbers();

    //
    // Channels settings
    //
    // Channel1
    //
    m_channels[CHANNEL_ONE].setup(pwmChan1, sensChan1, ledChan1);
    m_channels[CHANNEL_ONE].setFocus(true);

    //
    // Restore Channel ONE temperature from EEPROM, if crc8 match. Otherwise set it to TEMPERATURE_STANDBY
    // If in calibration mode, set it to TEMPERATURE_MIN
    //
    m_channels[CHANNEL_ONE].setTemperature(OPERATION_MODE_SET,
                                           (IS_DATA_ENABLED(DATA_IN_CALIBRATION) ? aDSChannel::TEMPERATURE_MIN :
                                                (_getTempFromEEPROM(EEPROM_ADDR_TEMP_CHANNEL_ONE, temp))
                                                ?
                                                    constrain(temp, aDSChannel::TEMPERATURE_MIN, aDSChannel::TEMPERATURE_MAX)
                                                :
                                                    aDSChannel::TEMPERATURE_STANDBY));
    m_channels[CHANNEL_ONE].syncTempChange();

    //
    // Channel2 enability pin
    //
    pinMode(chkChan2, INPUT);
    digitalRead(chkChan2);

#if 1
    //
    // Channel2 checking
    //
    if (digitalRead(chkChan2) == HIGH)
        _enableData(DATA_CHANNEL2_ENABLED, true);
#else
    _enableData(DATA_CHANNEL2_ENABLED, false);
#endif

    //
    // We force two channels in Calibration Mode
    //
    if (IS_DATA_ENABLED(DATA_IN_CALIBRATION))
        _enableData(DATA_CHANNEL2_ENABLED, true);

    if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
    {
        //
        // Get Channel joining state from EEPROM
        //
        int16_t start = EEPROM_ADDR_CHANNEL_JOINED;
        _read(joined, start);

        //
        // Set Channel 2 pins
        //
        m_channels[CHANNEL_TWO].setup(pwmChan2, sensChan2, ledChan2);

        //
        // Set relationship between channels (for PWM disabling while temp measurements)
        //
        m_channels[CHANNEL_ONE].setBrother(&m_channels[CHANNEL_TWO]);
        m_channels[CHANNEL_TWO].setBrother(&m_channels[CHANNEL_ONE]);

        //
        // Get Channel TWO temperature from EEPROM, if crc8 match. Otherwise set it to TEMPERATURE_STANDBY
        // If in calibration mode, set it to TEMPERATURE_MIN
        //
        m_channels[CHANNEL_TWO].setTemperature(OPERATION_MODE_SET,
                                               (IS_DATA_ENABLED(DATA_IN_CALIBRATION) ? aDSChannel::TEMPERATURE_MIN :
                                                    (_getTempFromEEPROM(EEPROM_ADDR_TEMP_CHANNEL_TWO, temp))
                                                    ?
                                                        constrain(temp, aDSChannel::TEMPERATURE_MIN, aDSChannel::TEMPERATURE_MAX)
                                                    :
                                                        aDSChannel::TEMPERATURE_STANDBY));
        //
        // If channels are joined, but temperature mismatche, set channel 2 temp from channel 1 one.
        // This could happen if uC is reset/turned off too early when join and temps are changed.
        //
        if (joined && (m_channels[CHANNEL_ONE].getTemperature(OPERATION_MODE_SET) != m_channels[CHANNEL_TWO].getTemperature(OPERATION_MODE_SET)))
            m_channels[CHANNEL_TWO].setTemperature(OPERATION_MODE_SET, m_channels[CHANNEL_ONE].getTemperature(OPERATION_MODE_SET));
        else
            m_channels[CHANNEL_TWO].syncTempChange();

        //
        // Enable Channel joining from stored value, except in Calibration Mode
        //
        _enableData(DATA_CHANNELS_JOINDED, IS_DATA_ENABLED(DATA_IN_CALIBRATION) ? false : joined);
    }

    //
    // Restore channel's calibration values
    //
    restoreCalibationValues();

    _showBanner();
}

/// \brief Set operation mode (SET/READ)
///
/// \param m OperationMode_t : new operation mode
/// \return void
///
///
void aDSChannels::setOperationMode(OperationMode_t m)
{
    OperationMode_t p = m_operationMode;

    m_operationMode = m;

    //
    // Set operation mode change, if necessary
    //
    _enableDataCheck(DATA_OPERATION, (p != m_operationMode));

    //
    // Set timestamp if current operation mode is OPERATION_MODE_SET, otherwise reset it.
    //
    m_operationTick = (m_operationMode == OPERATION_MODE_SET) ? millis() : 0;
}

/// \brief Get current operation mode
///
/// \return OperationMode_t : operation mode
///
///
OperationMode_t aDSChannels::getOperationMode()
{
    return m_operationMode;
}

/// \brief Update operation mode
///
/// If operation mode is currently set to OPERATION_MODE_SET, and OPERATION_SET_TIMEOUT timeout is
/// triggered, operation mode will be switched to OPERATION_MODE_READ
///
/// \return void
///
///
void aDSChannels::updateOperationMode()
{
    if (m_operationMode == OPERATION_MODE_SET)
    {
        if ((millis() - m_operationTick) > OPERATION_SET_TIMEOUT)
            setOperationMode(OPERATION_MODE_READ);
    }
}

/// \brief Reset timeout for OPERATION_MODE_SET mode
///
/// \return void
///
///
void aDSChannels::pingOperationMode()
{
    if (m_operationMode == OPERATION_MODE_SET)
        m_operationTick = millis();
}

/// \brief Enable a bit, if it's not already set, inside bitfield m_datas
///
/// \param bit uint16_t : bit to enable/disable
/// \param enable bool : bit enability
/// \return void
///
///
void aDSChannels::_enableDataCheck(uint16_t bit, bool enable)
{
    if (m_datas & bit)
        return;

    if (enable)
        m_datas |= bit;
    else
        m_datas &= (0xFFFF ^ bit);
}

/// \brief Enable a bit, regardless of its state, inside bitfield m_datas
///
/// \param bit uint16_t : bit to enable/disable
/// \param enable bool : bit enability
/// \return void
///
///
void aDSChannels::_enableData(uint16_t bit, bool enable)
{
    if (enable)
        m_datas |= bit;
    else
        m_datas &= (0xFFFF ^ bit);

}
#if 0 // Disabled: use IS_DATA_ENABLED() macro. YEAH, I KNOW ! :-D
bool aDSChannels::isDataEnabled(uint16_t bit)
{
    return (m_datas & bit);
}
#endif

/// \brief Reset given bit inside bitfield, regarless of its state
///
/// \param bit uint16_t : bit to reset
/// \return void
///
///
void aDSChannels::syncData(uint16_t bit)
{
    _enableData(bit, false);
}

/// \brief Increment or decrement encoder position
///
/// \param v uint16_t : increment value (signed)
/// \return void
///
///
void aDSChannels::incEncoderPosition(uint16_t v)
{
    //
    // We are in OPERATION_MODE_READ, so switch to OPERATION_MODE_SET and don't increase any temperature value
    //
    if (m_operationMode == OPERATION_MODE_READ)
    {
        setOperationMode(OPERATION_MODE_SET);
        //
        // Leave standby mode, if necessary
        //
        _wakeupFromStandby();

        return;
    }

    //
    // Leave standby mode, if necessary
    //
    _wakeupFromStandby();

    //
    // Store last time any temp has been changed
    //
    m_lastTempChange = millis();
    m_storedToEEPROM = false;

    //
    // Two Joined Channels or Single Channel Mode
    //
    if ((IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) && (IS_DATA_ENABLED(DATA_CHANNELS_JOINDED))) || !(IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED)))
    {
        Channel_t last = (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED)) ? CHANNEL_TWO : CHANNEL_ONE;

        //
        // Set new temperature to all channels
        //
        for (Channel_t c = CHANNEL_ONE; c <= last; c = static_cast<Channel_t>(c + 1))
        {
            aDSChannel *chan = &m_channels[c];
            int16_t temp = chan->getTemperature(m_operationMode);

            temp += v;

            temp = constrain(temp, aDSChannel::TEMPERATURE_MIN, aDSChannel::TEMPERATURE_MAX);

            _enableDataCheck((c == CHANNEL_ONE) ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL2_TEMP_SET, chan->setTemperature(m_operationMode, temp));
        }

    } // Two Channels Unjoined Mode
    else if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
    {
        //
        // Set new temperature to focused channel
        //
        aDSChannel *chan = m_channels[CHANNEL_ONE].hasFocus() ? &m_channels[CHANNEL_ONE] : &m_channels[CHANNEL_TWO];
        int16_t temp = chan->getTemperature(m_operationMode);

        temp += v;

        temp = constrain(temp, aDSChannel::TEMPERATURE_MIN, (IS_DATA_ENABLED(DATA_IN_CALIBRATION) ? 999 : aDSChannel::TEMPERATURE_MAX));

        _enableDataCheck((chan == &m_channels[CHANNEL_ONE]) ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL2_TEMP_SET, chan->setTemperature(m_operationMode, temp));
    }
}

/// \brief This member should be called often, it manage channels, LCD and EEPROM
///
/// \return void
///
///
void aDSChannels::service()
{
    bool storeToEEPROM = false;

    //
    // Temp value has changed, but aren't save to EEPROM, do it now
    //
    if (!m_storedToEEPROM && ((millis() - m_lastTempChange > TEMP_SETTING_INACTIVITY)))
    {
        //
        // Consider to store new temp values into EEPROM
        //
        storeToEEPROM = true;
    }

    unsigned long m = millis();

    //
    // It's time fo channels servicing
    //
    if ((m - m_nextMeasureUpdate) > MEASURE_UPDATE_RATE)
    {
        m_nextMeasureUpdate = m;

        _enableDataCheck(DATA_CHANNEL1_TEMP_READ, m_channels[CHANNEL_ONE].service(m));

        if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
            _enableDataCheck(DATA_CHANNEL2_TEMP_READ, m_channels[CHANNEL_TWO].service(m));
    }

    //
    // Update LED and LCD state for Channel 1
    //
    if (m_channels[CHANNEL_ONE].getLEDState() != m_channels[CHANNEL_ONE].updateLEDState(m))
        _enableData(DATA_CHANNEL1_LED_STATE, true);

    //
    // Update LED state for Channel 2
    //
    if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
    {
        if (m_channels[CHANNEL_TWO].getLEDState() != m_channels[CHANNEL_TWO].updateLEDState(m))
            _enableData(DATA_CHANNEL2_LED_STATE, true);
    }

    //
    // Need to save changed temp(s) to EEPROM
    //
    if (storeToEEPROM)
    {
        //
        // Store temperature to EEPROM only if we're not in calibration mode
        //
        if (! IS_DATA_ENABLED(DATA_IN_CALIBRATION))
        {
            if (m_channels[CHANNEL_ONE].isTempHasChanged())
            {
                _setTempToEEPROM(EEPROM_ADDR_TEMP_CHANNEL_ONE, m_channels[CHANNEL_ONE].getTemperature(OPERATION_MODE_SET));
                m_channels[CHANNEL_ONE].syncTempChange();
            }

            if (m_channels[CHANNEL_TWO].isTempHasChanged())
            {
                _setTempToEEPROM(EEPROM_ADDR_TEMP_CHANNEL_TWO, m_channels[CHANNEL_TWO].getTemperature(OPERATION_MODE_SET));
                m_channels[CHANNEL_TWO].syncTempChange();
            }
        }

        m_storedToEEPROM = true;
    }

    //
    // Update the LCD display
    //
    _updateDisplay();
}

/// \brief Toggle channels joining
///
/// \return void
///
///
void aDSChannels::toggleJoined()
{
    _wakeupFromStandby();

    //
    // Don't allow to join channels in Calibation Mode
    //
    if (IS_DATA_ENABLED(DATA_IN_CALIBRATION))
        return;

    if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
    {
        if (m_channels[CHANNEL_ONE].hasFocus())
            m_channels[CHANNEL_TWO].setTemperature(OPERATION_MODE_SET, m_channels[CHANNEL_ONE].getTemperature(OPERATION_MODE_SET));
        else
            m_channels[CHANNEL_ONE].setTemperature(OPERATION_MODE_SET, m_channels[CHANNEL_TWO].getTemperature(OPERATION_MODE_SET));

        _enableData(DATA_CHANNELS_JOINDED, !IS_DATA_ENABLED(DATA_CHANNELS_JOINDED));
        _enableData(DATA_DISPLAY, true);

        //
        // Store last time joined state changed
        //
        m_lastTempChange = millis();
        m_storedToEEPROM = false; // That will force to check if any temperature has changed, and save it to EEPROM, if necessary

        int16_t start = EEPROM_ADDR_CHANNEL_JOINED;
        _write(IS_DATA_ENABLED(DATA_CHANNELS_JOINDED), start);
    }
}

/// \brief Get channel joining state
///
/// \return bool : true if joinded, otherwise false
///
///
bool aDSChannels::isJoinded()
{
    return (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) ? IS_DATA_ENABLED(DATA_CHANNELS_JOINDED) : false);
}

/// \brief Set the focus to the next channel, if any
///
/// \return void
///
///
void aDSChannels::setFocusToNextChannel()
{
    _wakeupFromStandby();

    if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) && !IS_DATA_ENABLED(DATA_CHANNELS_JOINDED))
    {
        m_channels[CHANNEL_ONE].setFocus(!m_channels[CHANNEL_ONE].hasFocus());
        m_channels[CHANNEL_TWO].setFocus(!m_channels[CHANNEL_TWO].hasFocus());
        _enableDataCheck(DATA_DISPLAY, true);
        _enableDataCheck(DATA_FOCUS, true);
    }
}

/// \brief Toggle standby mode
///
/// \return void
///
///
void aDSChannels::toggleStandbyMode()
{
    _enableData(DATA_STANDBY, !IS_DATA_ENABLED(DATA_STANDBY));
    _enableDataCheck(DATA_DISPLAY_STANDBY, true);

    m_channels[CHANNEL_ONE].setStandbyMode(IS_DATA_ENABLED(DATA_STANDBY));

    if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
        m_channels[CHANNEL_TWO].setStandbyMode(IS_DATA_ENABLED(DATA_STANDBY));
}

/// \brief Get standby mode
///
/// \return bool : true if in standby mode, otherwise false
///
///
bool aDSChannels::isInStandby()
{
        return IS_DATA_ENABLED(DATA_STANDBY);
}

/// \brief Set calibration values for given channel
///
/// \param chan Channel_t : channel
/// \param cal aDSChannel::CalibrationData_t : calibration values
/// \return void
///
///
void aDSChannels::setCalibrationValues(Channel_t chan, aDSChannel::CalibrationData_t cal)
{
    m_channels[chan].setCalibration(cal.slope, cal.offset);
}

/// \brief Get calibration value for given channel
///
/// \param chan Channel_t : channel
/// \return aDSChannel::CalibrationData_t : calibration values
///
///
aDSChannel::CalibrationData_t aDSChannels::getCalibrationValues(Channel_t chan)
{
    return m_channels[chan].getCalibration();
}

/// \brief Restore calibration values from EEPOM
///
/// \return void
///
///
void aDSChannels::restoreCalibationValues()
{
    _restoreCalibrationFromEEPROM(EEPROM_ADDR_CALIBRATION_CHAN_1, m_channels[CHANNEL_ONE]);

    if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
        _restoreCalibrationFromEEPROM(EEPROM_ADDR_CALIBRATION_CHAN_2, m_channels[CHANNEL_TWO]);

    if (IS_DATA_ENABLED(DATA_IN_CALIBRATION))
    {
        aDSChannel::CalibrationData_t cal = { .slope = 1.0, .offset = 0.0 };

        setCalibrationValues(aDSChannels::CHANNEL_ONE, cal);

        if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
            setCalibrationValues(aDSChannels::CHANNEL_TWO, cal);

    }
}

/// \brief Save calibation value of given into EEPROM
///
/// \param chan Channel_t : channel
/// \return void
///
///
void aDSChannels::saveCalibrationValues(Channel_t chan)
{
    _backupCalibrationFromEEPROM(chan == CHANNEL_ONE ? EEPROM_ADDR_CALIBRATION_CHAN_1 : EEPROM_ADDR_CALIBRATION_CHAN_2, m_channels[chan]);
}

/// \brief Get calibration mode enability
///
/// \return bool : true if in calibration mode, otherwise false
///
///
bool aDSChannels::isInCalibration()
{
    return IS_DATA_ENABLED(DATA_IN_CALIBRATION);
}

/// \brief Set calibration mode
///
/// \param enable bool : enability
/// \return void
///
///
void aDSChannels::setCalibrationMode(bool enable)
{
    _enableData(DATA_IN_CALIBRATION, enable);

    if (!enable)
        _enableData(DATA_DISPLAY, true);
}

/// \brief Wake up from standby mode
///
/// \return void
///
///
void aDSChannels::_wakeupFromStandby()
{
    if (IS_DATA_ENABLED(DATA_STANDBY))
        toggleStandbyMode();
}

/// \brief Display a big digit to given position
///
/// \param digit uint8_t : offset in _bigDigit{Top/Bottom} array
/// \param position uint8_t : LCD position
/// \param offset uint8_t : LCD offset position
/// \return void
///
///
void aDSChannels::_displayBigDigit(uint8_t digit, uint8_t position, uint8_t offset)
{
    //
    // Upper part digit
    //
    m_lcd.setCursor((position * (DIGIT_WIDTH + 1)) + offset, 0);
    for(uint8_t i = 0; i < DIGIT_WIDTH; i++)
        m_lcd.write(_bigDigitsTop[digit][i]);

    //
    // Lower part digit
    //
    m_lcd.setCursor((position * (DIGIT_WIDTH + 1)) + offset, 1);
    for(uint8_t i = 0; i < DIGIT_WIDTH; i++)
        m_lcd.write(_bigDigitsBottom[digit][i]);
}

/// \brief Display a big digits number to given position
///
/// \param value uint16_t : value to display, DIGIT_WIDTH max length
/// \param position uint8_t : LCD position
/// \param offset uint8_t : LCD offset position
/// \return void
///
///
void aDSChannels::_displayBigDigits(int16_t value, uint8_t position, uint8_t offset)
{
    uint8_t     buffer[5];
    uint8_t    *p = &buffer[0];
    uint8_t     index = 0;

    if (value >= 0)
    {
        //
        // Right Align
        //
        if (value < 100)
            *p++ = ':'; // empty big char
        if (value < 10)
            *p++ = ':'; // empty big char

        itoa(value, (char *)p, 10);
    }
    else // value == -1
    {
        *p++ = ';';
        *p++ = ';';
        *p++ = ';';
        *p++ = '\0';
    }

    //
    // Display each digit in sequence
    //
    for(; index < (sizeof(buffer) - 1); index++)
    {
        uint8_t c = buffer[index];

        if (c == 0)
            break;

        c = c - 48;

        _displayBigDigit(c, position + index, offset);
    }

    //
    // Display degree char, or nothing if value < 0
    //
    m_lcd.setCursor((position + index) * (DIGIT_WIDTH + 1) + offset, 0);
    m_lcd.write((value >= 0) ? 0xDF : 0x20); // ° character
}

/// \brief Clear numerical value field (in non big digit mode) on LCD
///
/// \param row uint8_t : LCD row position
/// \param destMinus int : right offset sub
/// \return void
///
///
void aDSChannels::_clearValue(uint8_t row, int destMinus)
{
    m_lcd.setCursor(OFFSET_VALUE, row);
    for (uint8_t i = OFFSET_VALUE; i < OFFSET_MARKER_RIGHT + destMinus; i++)
        m_lcd.write(char(0x20));
}

/// \brief Update value on LCD from given mode and row
///
/// \param mode OperationMode_t : operation mode (SET/READ)
/// \param value int16_t : value to display
/// \param row uint8_t : LCD row
/// \return void
///
///
void aDSChannels::_updateField(OperationMode_t mode, int16_t value, uint8_t row)
{
    _clearValue(row);

    if (value == -1)
    {
        m_lcd.setCursor(((mode == OPERATION_MODE_READ) ? OFFSET_VALUE : (OFFSET_MARKER_RIGHT - 3) - 2), row);
        m_lcd.print(F("---"));
    }
    else
    {
        m_lcd.setCursor(((mode == OPERATION_MODE_READ) ? OFFSET_VALUE : (OFFSET_MARKER_RIGHT - getNumericalLength(value)) - 2), row);
        m_lcd.print(value, DEC);

        if (!IS_DATA_ENABLED(DATA_IN_CALIBRATION) || (IS_DATA_ENABLED(DATA_IN_CALIBRATION) && (mode == OPERATION_MODE_SET)))
            m_lcd.write(0xDF); // ° like
    }
}

/// \brief Update LCD display, if needed
///
/// \return void
///
///
void aDSChannels::_updateDisplay()
{
    bool            fullRedraw = false;
    unsigned long   m = millis();

    //
    // Update the display each DISPLAY_UPDATE_RATE ms
    //
    if ((m_operationMode == OPERATION_MODE_SET)
        || IS_DATA_ENABLED(DATA_FOCUS)
        || (IS_DATA_ENABLED(DATA_CHANNEL1_LED_STATE) || IS_DATA_ENABLED(DATA_CHANNEL2_LED_STATE))
        || (m - m_nextDisplayUpdate) > DISPLAY_UPDATE_RATE)
    {
        m_nextDisplayUpdate = m;

        //
        // Single or joined.
        //
        if ((IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) && (IS_DATA_ENABLED(DATA_CHANNELS_JOINDED))) || !(IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED)))
        {
            //
            // Operation mode has changed
            //
            if (IS_DATA_ENABLED(DATA_OPERATION))
            {
                fullRedraw = true;
                syncData(DATA_OPERATION);
            }

            //
            // Full redraw needed
            //
            if ((IS_DATA_ENABLED(DATA_DISPLAY)) || fullRedraw)
            {
                fullRedraw = true;
                syncData(DATA_DISPLAY);

                m_lcd.clear();
            }

            //
            // Does channels plugged ?
            //
            bool chan1Plugged = m_channels[CHANNEL_ONE].isPlugged();
            bool chan2Plugged = IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) ? m_channels[CHANNEL_TWO].isPlugged() : false;

            //
            // Displayed temperature of channel 1 has changed
            //
            if (  (IS_DATA_ENABLED(((m_operationMode == OPERATION_MODE_SET) ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL1_TEMP_READ))
                  || // Or displayed temperature of channel 2 has changed
                  (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) &&  IS_DATA_ENABLED(((m_operationMode == OPERATION_MODE_SET) ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL2_TEMP_READ))))
                || // Or full redraw needed
                  fullRedraw)
            {
                int16_t chan1Temp = chan1Plugged ? m_channels[CHANNEL_ONE].getTemperature(OPERATION_MODE_READ) : -1;
                int16_t chan2Temp = IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) && chan2Plugged ? m_channels[CHANNEL_TWO].getTemperature(OPERATION_MODE_READ) : -1;
                int16_t averagedTemp;

                if (chan1Plugged && chan2Plugged)
                    averagedTemp = (chan1Temp + chan2Temp) >> 1;
                else if (chan1Plugged)
                    averagedTemp = chan1Temp;
                else if (chan2Plugged)
                    averagedTemp = chan2Temp;
                else
                    averagedTemp = -1;


                int16_t t = (m_operationMode == OPERATION_MODE_READ) ? averagedTemp : m_channels[CHANNEL_ONE].getTemperature(m_operationMode);

                _displayBigDigits(t, 0, (m_operationMode == OPERATION_MODE_SET) ? 2 : 0);
                syncData(m_operationMode == OPERATION_MODE_SET ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL1_TEMP_READ);
                syncData(m_operationMode == OPERATION_MODE_SET ? DATA_CHANNEL2_TEMP_SET : DATA_CHANNEL2_TEMP_READ);
            }

        } // Two separate channels
        else if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED))
        {
            //
            // Operation mode has changed
            //
            if (IS_DATA_ENABLED(DATA_OPERATION))
            {
                fullRedraw = true;
                syncData(DATA_OPERATION);
            }

            //
            // Full redraw needed
            //
            if ((IS_DATA_ENABLED(DATA_DISPLAY)) || fullRedraw)
            {
                fullRedraw = true;
                syncData(DATA_DISPLAY);

                m_lcd.clear();
            }

            //
            // Notify in calibration mode
            //
            if (IS_DATA_ENABLED(DATA_IN_CALIBRATION) && fullRedraw)
            {
                m_lcd.setCursor(OFFSET_MARKER_RIGHT + 1, 0);
                m_lcd.print(F("CAL"));
            }

            //
            // Displayed temperature of channel 1 has changed, or full redraw
            //
            if ((IS_DATA_ENABLED(((m_operationMode == OPERATION_MODE_SET) ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL1_TEMP_READ))) || fullRedraw)
            {
                int16_t t = (IS_DATA_ENABLED(DATA_IN_CALIBRATION) && (m_operationMode == OPERATION_MODE_READ))
                                ? m_channels[CHANNEL_ONE].getADCValue(): m_channels[CHANNEL_ONE].getTemperature(m_operationMode);

                _updateField(m_operationMode, ((m_operationMode == OPERATION_MODE_READ)
                                                ?
                                                    (IS_DATA_ENABLED(DATA_IN_CALIBRATION) ? t : (m_channels[CHANNEL_ONE].isPlugged() ? t : -1) )
                                                :
                                                    t)
                                            , 0);
                syncData(m_operationMode == OPERATION_MODE_SET ? DATA_CHANNEL1_TEMP_SET : DATA_CHANNEL1_TEMP_READ);
            }

            //
            // Displayed temperature of channel 2 has changed, or full redraw
            //
            if ((IS_DATA_ENABLED(((m_operationMode == OPERATION_MODE_SET) ? DATA_CHANNEL2_TEMP_SET : DATA_CHANNEL2_TEMP_READ))) || fullRedraw)
            {
                int16_t t = (IS_DATA_ENABLED(DATA_IN_CALIBRATION) && (m_operationMode == OPERATION_MODE_READ))
                                ? m_channels[CHANNEL_TWO].getADCValue(): m_channels[CHANNEL_TWO].getTemperature(m_operationMode);

                _updateField(m_operationMode, ((m_operationMode == OPERATION_MODE_READ)
                                               ?
                                                    (IS_DATA_ENABLED(DATA_IN_CALIBRATION) ? t : (m_channels[CHANNEL_TWO].isPlugged() ? t : -1) )
                                               :
                                                     t)
                                            , 1);
                syncData(m_operationMode == OPERATION_MODE_SET ? DATA_CHANNEL2_TEMP_SET : DATA_CHANNEL2_TEMP_READ);
            }

            //
            // Channel focus has changed
            //
            if ((IS_DATA_ENABLED(DATA_FOCUS)) || fullRedraw)
            {
                bool focus = m_channels[CHANNEL_ONE].hasFocus();

                m_lcd.setCursor(OFFSET_MARKER_LEFT, focus);
                m_lcd.write(' ');
                m_lcd.setCursor(OFFSET_MARKER_RIGHT, focus);
                m_lcd.write(' ');

                m_lcd.setCursor(OFFSET_MARKER_LEFT, !focus);
                m_lcd.write('[');
                m_lcd.setCursor(OFFSET_MARKER_RIGHT, !focus);
                m_lcd.write(']');

                syncData(DATA_FOCUS);
            }
        }

        //
        // Standby mode has been toggled
        //
        if ((IS_DATA_ENABLED(DATA_DISPLAY_STANDBY)) || fullRedraw)
        {
            if (!(IS_DATA_ENABLED(DATA_CHANNELS_JOINDED)) || ((IS_DATA_ENABLED(DATA_CHANNELS_JOINDED)) && (m_operationMode == OPERATION_MODE_READ)))
            {
#ifndef LCD_CHANNELS_LEDS
                m_lcd.setCursor(m_lcdCols - 4, 1);
                m_lcd.print(IS_DATA_ENABLED(DATA_STANDBY) ? F("Stby") : F("    "));
#endif

                syncData(DATA_DISPLAY_STANDBY);
            }
        }

        //
        // Redraw LCD LED for Channel 1
        //
        if (IS_DATA_ENABLED(DATA_CHANNEL1_LED_STATE) || fullRedraw)
        {
#ifdef LCD_CHANNELS_LEDS
            m_lcd.setCursor(m_lcdCols - 1, 0);
            m_lcd.write(m_channels[CHANNEL_ONE].getLEDState() == HIGH ? 0x7 : ' ');
#endif
            syncData(DATA_CHANNEL1_LED_STATE);
        }

        //
        // Redraw LCD LED for Channel 2
        //
        if (IS_DATA_ENABLED(DATA_CHANNEL2_ENABLED) && (IS_DATA_ENABLED(DATA_CHANNEL2_LED_STATE) || fullRedraw))
        {
#ifdef LCD_CHANNELS_LEDS
            m_lcd.setCursor(m_lcdCols - 1, 1);
            m_lcd.write(m_channels[CHANNEL_TWO].getLEDState() == HIGH ? 0x7 : ' ');
#endif
            syncData(DATA_CHANNEL2_LED_STATE);
        }
    }
}
//
// End of Class aDSChannels
//

//
// Begin of Class aDSEngine
//
/// \brief aDSEngine constructor
///
aDSEngine::aDSEngine() :
    m_channels(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN),
    m_encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_PB_PIN, ENCODER_STEPS_PER_NOTCH),
    m_RXoffset(0),
    m_serialInputTick(0)
{
    //ctor

    //
    // If encoder's push button is pressed on startup, we turn Calibration Mode ON
    //
    if (digitalRead(ENCODER_PB_PIN) == LOW)
    {
        m_channels.setCalibrationMode(true);

        //while (digitalRead(ENCODER_PB_PIN) == LOW)
        //    delay(10);
    }
}

/// \brief aDSEngine destructor
///
aDSEngine::~aDSEngine()
{
    //dtor
}

/// \brief Timer1 class ISR function
///
/// This function is periodically called from Timer1 class, which handles encoder events
///
/// \return void
///
///
void timer1ISR(void)
{
    pEncoder->service();
}

/// \brief Handle serial input, in calibation mode only
///
/// \return void
///
///
void aDSEngine::_handleSerialInput()
{
    if (Serial)
    {
        if (Serial.available() > 0)
        {
            bool EOL = false;

            while (Serial.available() > 0)
            {
                m_RXbuffer[m_RXoffset] = Serial.read();

                if ((m_RXbuffer[m_RXoffset] == 0xA) || (m_RXoffset == (RXBUFFER_MAXLEN - 1) /* Overflow checking */))
                {
                    EOL = true;
                    break;
                }

                m_RXoffset++;
            }

            if (EOL)
            {
                bool valid = false;

                m_RXbuffer[m_RXoffset] = '\0';

                if (m_RXoffset >= 3)
                {
                    /*
                     *** Format is: :CMD:<ARG> or :CMD:SUB:ARG
                     ***         CMD is command
                     ***         SUB is sub-command
                     ***         ARG is (<>: optional) argument
                     */

                    uint8_t *cmdStart, *cmdEnd;
                    if (((cmdStart = (uint8_t *)strchr((const char *)&m_RXbuffer[0], ':')) != NULL) &&
                        ((cmdEnd = (uint8_t *)strchr((const char *)cmdStart + 1, ':')) != NULL) )
                    {
                        uint8_t cmd[(cmdEnd - cmdStart)];
                        uint8_t *arg = cmdEnd + 1;

                        memcpy(&cmd[0], cmdStart + 1, sizeof(cmd) - 1);
                        cmd[sizeof(cmd) - 1] = '\0';

                        Serial.print(':');

                        if (strcmp_P((const char *)cmd, PSTR("CAL")) == 0) // Calibration
                        {
                            valid = true;

                            if (strcmp_P((const char *)arg, PSTR("OFF")) == 0) // Off : no calibation value stored into EEPROM
                            {
                                m_channels.setCalibrationMode(false);
                                m_channels.restoreCalibationValues();
                            }
                            else if (strcmp_P((const char *)arg, PSTR("SAVE")) == 0) // Store calibation value into EEPROM, then leave calibration mode
                            {
                                m_channels.setCalibrationMode(false);
                                m_channels.saveCalibrationValues(aDSChannels::CHANNEL_ONE);
                                m_channels.saveCalibrationValues(aDSChannels::CHANNEL_TWO);
                            }
                            else if (strcmp_P((const char *)arg, PSTR("DUMP")) == 0) // Dump calibration values
                            {
                                aDSChannel::CalibrationData_t   cal;
                                aDSChannel                      chans[aDSChannels::CHANNEL_MAX];

                                // Retrieve calibration values stored in EEPROM for channel 1
                                m_channels._restoreCalibrationFromEEPROM(aDSChannels::EEPROM_ADDR_CALIBRATION_CHAN_1, chans[aDSChannels::CHANNEL_ONE]);
                                cal = chans[aDSChannels::CHANNEL_ONE].getCalibration();

                                Serial.print(F("STORED:"));
                                Serial.print(F("1="));
                                Serial.print(cal.slope, 9);
                                Serial.print(F(","));
                                Serial.print(cal.offset, 9);

                                // Retrieve calibration values stored in EEPROM for channel 2
                                m_channels._restoreCalibrationFromEEPROM(aDSChannels::EEPROM_ADDR_CALIBRATION_CHAN_2, chans[aDSChannels::CHANNEL_TWO]);
                                cal = chans[aDSChannels::CHANNEL_TWO].getCalibration();
                                Serial.print(F(";2="));
                                Serial.print(cal.slope, 9);
                                Serial.print(F(","));
                                Serial.print(cal.offset, 9);

                                Serial.print(F(":CURRENT:"));

                                cal = m_channels.getCalibrationValues(aDSChannels::CHANNEL_ONE);
                                Serial.print(F("1="));
                                Serial.print(cal.slope, 9);
                                Serial.print(F(","));
                                Serial.print(cal.offset, 9);
                                cal = m_channels.getCalibrationValues(aDSChannels::CHANNEL_TWO);
                                Serial.print(F(";2="));
                                Serial.print(cal.slope, 9);
                                Serial.print(F(","));
                                Serial.print(cal.offset, 9);
                            }
                            else // Handle calibration values: ':CAL:x:slope,offset'
                            {
                                uint8_t *pV1; uint8_t *pV2;

                                if (((pV1 = (uint8_t *)strchr((const char *)arg, ':')) != NULL)
                                    && ((pV2 = (uint8_t *)strchr((const char *)pV1, ',')) != NULL))
                                {
                                    pV1++;

                                    aDSChannel::CalibrationData_t cal;
                                    uint8_t cmd2[(pV1 - arg)];

                                    memcpy(&cmd2[0], arg, sizeof(cmd2) - 1);
                                    cmd2[sizeof(cmd2) - 1] = '\0';

                                    *pV2++ = '\0';

                                    cal.slope = atof((char *)pV1);
                                    cal.offset = atof((char *)pV2);

                                    if (strcmp_P((const char *)cmd2, PSTR("1")) == 0)
                                        m_channels.setCalibrationValues(aDSChannels::CHANNEL_ONE, cal);
                                    else if (strcmp_P((const char *)cmd2, PSTR("2")) == 0)
                                        m_channels.setCalibrationValues(aDSChannels::CHANNEL_TWO, cal);
                                    else
                                        valid = false;
                                }
                                else
                                    valid = false;
                            }
                        }
                        else
                            Serial.print(F("INVALID"));

                    }
                }

                //
                // Clear buffer for next command
                //
                m_RXoffset = 0;

                Serial.print(valid ? F(":OK:\r\n") : F(":ERR:\r\n"));
            }
        }
    }
}

/// \brief Setup member function, should be called before any other member
///
/// \return void
///
///
void aDSEngine::setup()
{
    //
    // Setup channels
    //
    m_channels.setup(LCD_COLS, LCD_ROWS,
                     PWM_CHANNEL1_PIN, TEMP_SENSOR_CHANNEL1_PIN, LED_CHANNEL1_PIN,
                     CHANNEL2_ENABLE_PIN,
                     PWM_CHANNEL2_PIN, TEMP_SENSOR_CHANNEL2_PIN, LED_CHANNEL2_PIN);

    //
    // Pointer to ClickEncoder object (used in timer1ISR())
    //
    pEncoder = &m_encoder;

    //
    // Initialize Timer1
    //
    Timer1.initialize(500);
    Timer1.attachInterrupt(timer1ISR);
}

/// \brief Main loop
///
/// \return void
///
///
void aDSEngine::run()
{
    unsigned long m;

    //
    // Initialize operation mode timeout
    //
    m_channels.pingOperationMode();

    while(1)
    {
        //
        // Get the encoder detents value
        //
        int16_t v = m_encoder.getValue();

        //
        // Check if we need to go back to OPERATION_MODE_READ
        //
        m_channels.updateOperationMode();

        //
        // Encoder has been rotated
        //
        if (v != 0)
        {
            m_channels.incEncoderPosition(v);
            m_channels.pingOperationMode();
        }

        //
        // Any encoder push button event ?
        //
        ClickEncoder::Button b = m_encoder.getButton();
        if (b != ClickEncoder::Open)
        {
            switch (b)
            {
                case ClickEncoder::Clicked: // Change channel focus
                    m_channels.setFocusToNextChannel();
                    break;

                case ClickEncoder::DoubleClicked: // Toggle standby mode
                    m_channels.toggleStandbyMode();
                    break;

                case ClickEncoder::Held: // Toggle joined channels
                    m_channels.toggleJoined();

                    //
                    // Wait while button is held
                    //
                    do
                    {
                        m_channels.service();
                    } while (m_encoder.getButton() == ClickEncoder::Held);
                    break;

                default:
                    break;
            }
        }

        //
        // Servicing channels
        //
        m_channels.service();

        //
        // Handle serial input (in calibration mode ONLY, time is precious)
        //
        if (m_channels.isInCalibration() && (((m = millis()) - m_serialInputTick) > 300))
        {
            m_serialInputTick = m;
            _handleSerialInput();
        }
    }
}
//
// End of Class aDSEngine
//
