/*
	Debug can be enabled optionally by defining the following
	variable. Note that debug and deep-sleep currently
	is not compatible
*/
// #define DEBUG 1

/*
	Periodic DHT-22 temperature and humidity measurement
	with BSFrance LoRA32u4 II
*/

#include <DHT.h>
#include <stdint.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>

/*
	Transmission interval in seconds
	(Default interval / hardcoded interval)

	Min and max values are used for dynamic reconfiguration
	during downlink

	TX_INTERVAL is the default transmission interval.

	ToDo: Currently storage of current TX_INTERVAL in
	      EEPROM is not implemented (coming soon)
*/
#define TX_INTERVAL				900
#define TX_INTERVAL_MIN			900
#define TX_INTERVAL_MAX			14400

static uint16_t txIntervalCurrent;

/*
	Select message format.
	
	Short:		10 bit temperature (range -20 to 82.4 Celsius)
				10 bit humidity (0% to 102.4%)
				4  bit voltage (0 to 3.3V, 206.25mV steps)
*/
#define MESSAGEFORMAT_SHORT		0
#define MESSAGEFORMAT MESSAGEFORMAT_SHORT

/*
	Enable (1) or disable (0) deep-sleep between measurements
*/
#define ENABLEDEEPSLEEP 1

/*
	DHT22 configuration
*/
#define DHTPIN					10
#define DHTTYPE					DHT22

/*
	Analog port (battery voltage) configuration
*/
#define ANALOGPIN        9

/* Singleton for temp & hum. sensor */
DHT dhtSensor(DHTPIN, DHTTYPE);

/*
        LoRA configuration
*/

// LSB:
static const u1_t PROGMEM DEVEUI[8] = /*{FILLIN_LORADEVEUI}*/ 		/* { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; */
static const u1_t PROGMEM APPEUI[8] = /*{FILLIN_LORAAPPEUI}*/		/* { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; */

// MSB:
static const u1_t PROGMEM APPKEY[16] = /*{FILLIN_LORAAPPKEY}*/ 		/* { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; */

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);  }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);  }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

#if MESSAGEFORMAT == MESSAGEFORMAT_SHORT
	static uint8_t messageBytes[3];
#endif
static osjob_t sendjob;
#if ENABLEDEEPSLEEP == 1
	static bool txPending;
	static uint32_t sleepPeriod;
#endif

/*
	Pin mapping for RFM95(W) module.

	Depending on board revision you have to include
	a solder bridge or connect dead 0 ohm resistors
	on the backside!
*/
const lmic_pinmap lmic_pins = {
	.nss = 8,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = 4,
	.dio = {7, 6, LMIC_UNUSED_PIN},
};


void onEvent(ev_t ev) {
	#ifdef DEBUG
		Serial.print(os_getTime());
		Serial.print(": ");
	#endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
			#ifdef DEBUG
				Serial.println(F("EV_SCAN_TIMEOUT"));
			#endif
            break;
        case EV_BEACON_FOUND:
			#ifdef DEBUG
				Serial.println(F("EV_BEACON_FOUND"));
			#endif
            break;
        case EV_BEACON_MISSED:
			#ifdef DEBUG
				Serial.println(F("EV_BEACON_MISSED"));
			#endif
            break;
        case EV_BEACON_TRACKED:
			#ifdef DEBUG
				Serial.println(F("EV_BEACON_TRACKED"));
			#endif
            break;
        case EV_JOINING:
			#ifdef DEBUG
				Serial.println(F("EV_JOINING"));
			#endif
            break;
        case EV_JOINED:
			#ifdef DEBUG
				Serial.println(F("EV_JOINED"));
			#endif

			/*
				Disable link check validation (not supported on TTN)
			*/
            LMIC_setLinkCheckMode(0);
            break;

		case EV_RFU1:
			#ifdef DEBUG
				Serial.println(F("EV_RFU1"));
			#endif
			break;
        case EV_JOIN_FAILED:
			#ifdef DEBUG
				Serial.println(F("EV_JOIN_FAILED"));
			#endif
			/* Just ignore - we will join again the next intervall */
			LMIC_reset();
			LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
			#if ENABLEDEEPSLEEP == 0
				os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(txIntervalCurrent), do_send);
			#else
				txPending = false;
				sleepPeriod = txIntervalCurrent/8;
			#endif                        
            break;
        case EV_REJOIN_FAILED:
			#ifdef DEBUG
				Serial.println(F("EV_REJOIN_FAILED"));
			#endif
			/* Just ignore - we will join again the next intervall */
			LMIC_reset();
			LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
			#if ENABLEDEEPSLEEP == 0
				os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(txIntervalCurrent), do_send);
			#else
				txPending = false;
				sleepPeriod = txIntervalCurrent/8;
			#endif
            break;
        case EV_TXCOMPLETE:
			#ifdef DEBUG
				Serial.println(F("EV_TXCOMPLETE")); // (includes waiting for RX windows)

				if (LMIC.txrxFlags & TXRX_ACK) {
					Serial.println(F("Received ack"));
				}
				if (LMIC.dataLen) {
					Serial.println(F("Received "));
					Serial.println(LMIC.dataLen);
					Serial.println(F(" bytes of payload"));
				}
			#endif

			/*
				Check if our broadcast interval has been changed
			*/
			if(LMIC.dataLen > 0) {
				#ifdef DEBUG
					Serial.print("Got downstream data ... ");
				#endif
				if(LMIC.dataLen == 2) {
					#ifdef DEBUG
						Serial.println("size is valid");
					#endif
					uint16_t newInterval = (((uint16_t)(LMIC.frame[LMIC.dataBeg + 0])) & 0x00FF) | ((((uint16_t)(LMIC.frame[LMIC.dataBeg + 0])) << 8) & 0xFF00);

					// Update our local interval variable AND write to eeprom if the value was valid
					if((newInterval >= TX_INTERVAL_MIN) && (newInterval <= TX_INTERVAL_MAX)) {
						txIntervalCurrent = newInterval;
						#ifdef DEBUG
							Serial.print("New TX interval: ");
							Serial.print(txIntervalCurrent);
							Serial.println(" seconds");
						#endif
						// ToDo: Update EEPROM
					} else {
						#ifdef DEBUG
							Serial.println("Requested interval is out of range");
						#endif
					}
				}
			}

			/*
				Shedule next transmission
			*/
			#if ENABLEDEEPSLEEP == 0
				os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(txIntervalCurrent), do_send);
			#else
				txPending = false;
				sleepPeriod = txIntervalCurrent/8;
			#endif
			break;
        case EV_LOST_TSYNC:
			#ifdef DEBUG
				Serial.println(F("EV_LOST_TSYNC"));
			#endif
            break;
        case EV_RESET:
			#ifdef DEBUG
				Serial.println(F("EV_RESET"));
			#endif
            break;
        case EV_RXCOMPLETE:
            #ifdef DEBUG
				Serial.println(F("EV_RXCOMPLETE in ping slot"));
			#endif
            break;
        case EV_LINK_DEAD:
			#ifdef DEBUG
				Serial.println(F("EV_LINK_DEAD"));
			#endif
            break;
        case EV_LINK_ALIVE:
			#ifdef DEBUG
				Serial.println(F("EV_LINK_ALIVE"));
			#endif
            break;
         default:
			#ifdef DEBUG
				Serial.println(F("Unknown event"));
			#endif
            break;
    }
}

static void do_send(osjob_t* j) {
	float humidity;
	float temperature;
	int voltageRaw;

	/*
		Check if there is currently a send job pending. If it is,
		do not transmit anything. Else queue the next packet.
	*/
    if(!(LMIC.opmode & OP_TXRXPEND)) {
		/*
			Read data from DHT22
		*/
		humidity = dhtSensor.readHumidity();
		temperature = dhtSensor.readTemperature();
		voltageRaw = analogRead(ANALOGPIN);

		/*
			Build package
		*/
		#if MESSAGEFORMAT == MESSAGEFORMAT_SHORT
			uint32_t msg = (((uint32_t)((temperature + 20.0) * 10.0)) & 0x03FF)
						| ((((uint32_t)(humidity * 10.0)) & 0x03FF) << 10)
						| (((((uint32_t)voltageRaw) >> 6) & 0x0F) << 20);
			messageBytes[0] = (uint8_t)((msg >> 0) & 0xFF);
			messageBytes[1] = (uint8_t)((msg >> 8) & 0xFF);
			messageBytes[2] = (uint8_t)((msg >> 16) & 0xFF);

			LMIC_setTxData2(1,  messageBytes, 3, 0);

			#if ENABLEDEEPSLEEP == 1
				txPending = true;
			#endif
		#endif
		#ifdef DEBUG
			Serial.println(F("Packet queued"));
		#endif
    }
}

void setup() {
	#ifdef DEBUG
		delay(2500);
	#endif
	Serial.begin(115600);
	#ifdef DEBUG
		/*
			In debug mode await serial console
			attachment. Note that this means that
			nodes in debug mode are NOT capable
			of booting and running without the
			console being attached!
		*/
		while (!Serial) { delay(1); }
	#endif
	/*
		Initialize DHT
	*/
	dhtSensor.begin();

	/*
		Initialize ADC for voltage measurement on battery.
		Disable internal pullups.
	*/
	pinMode(ANALOGPIN, INPUT);
	digitalWrite(ANALOGPIN, 0);
        
	#if ENABLEDEEPSLEEP == 1
		txPending = true;
	#endif

	/*
		Include a short delay before LoRA
		initialization.
	*/
	txIntervalCurrent = TX_INTERVAL;
    // ToDo: Read from EEPROM
	delay(500);

	/*
		LoRA initialization
	*/
	os_init();
	LMIC_reset();
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

	/*
		Execute first transmission. This also triggers activation
	*/
	do_send(&sendjob);
}

/*
        Main loop
*/
void loop() {
	#if ENABLEDEEPSLEEP == 0
		os_runloop_once();
	#else
		if(txPending) {
			os_runloop_once();
		} else {
			/*
				Do a sleep period in deep sleep mode OR start a send job
			*/
			if((sleepPeriod = sleepPeriod - 1) == 0) {
				/*
					Slept enough. Reinitialize LMIC and continue
				*/
				txPending = true;
				do_send(&sendjob);
			} else {
				/*
					We continue sleeping
				*/
				#ifdef DEBUG
					Serial.print(".");
					Serial.flush(); // Just in case there is pending data
					Serial.end();
					delay(500);
				#endif
				LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
				#ifdef DEBUG
					Serial.begin(115600);
					delay(500);
				#endif
			}
		}
	#endif
}
