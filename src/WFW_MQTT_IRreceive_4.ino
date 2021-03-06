//Project to control the IR devices by a HOMIE instance.
//21.08.2017 @ WFW:		Temperature is measured and transmitted via MQTT
//						IR receiving and sending works.

/* 	/sensor/ircontrol/$homie 2.0.0
	/sensor/ircontrol/$implementation esp8266
	/sensor/ircontrol/$implementation/config {"wifi":{"ssid":"ZKb20b"},"mqtt":{"host":"192.168.178.26","port":1883,"base_topic":"/sensor/",           "auth":false},"name":"IR control","ota":{"enabled":true},"device_id":"ircontrol"}
	/sensor/ircontrol/$implementation/version 2.0.0
	/sensor/ircontrol/$implementation/ota/enabled true
	/sensor/ircontrol/$mac 5C:CF:7F:2C:8E:71
	/sensor/ircontrol/IR1/$type IR send/receive
	/sensor/ircontrol/IR1/$properties IRreceived,temperature,cmd:settable
	/sensor/ircontrol/IR1/temperature 21.87
	/sensor/ircontrol/IR1/IRreceived 20DF10EF
	/sensor/ircontrol/$name IR control
	/sensor/ircontrol/$localip 192.168.178.51
	/sensor/ircontrol/$stats/interval 60
	/sensor/ircontrol/$stats/signal 72
	/sensor/ircontrol/$stats/uptime 601343
	/sensor/ircontrol/$fw/name NaundorfIT_IR
	/sensor/ircontrol/$fw/version 1.1
	/sensor/ircontrol/$fw/checksum b59cfe819318f164f64bf9824ce6113a
	/sensor/ircontrol/$online true
	/sensor/ircontrol/$stats/signal 70
	/sensor/ircontrol/$stats/uptime 601403
 */

#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
HomieNode IRdeviceDriver("IR1", "IR send/receive");  // The first parameter defines the name of the 
//                                                      device. The second comments the type of the device.

#define ONE_WIRE_BUS D6					// Data wire is plugged into port GPIO12 on the Arduino
OneWire oneWire(ONE_WIRE_BUS);			// Setup a oneWire instance
DallasTemperature sensors(&oneWire);	// Pass our oneWire reference to Dallas Temperature. 

int RECV_PIN = D2;				//IR detector connected to GPIO pin 4 (D2 on NodeMCU)
int SEND_PIN = D1;				//Transistor with IR LED on GPIO pin 5 (D1 on NodeMCU)		   
IRrecv irrecv(RECV_PIN);		// Setup IR remote instance
decode_results results;			//
IRsend irsend(SEND_PIN);		//

long now = 0; //in ms
float temp = 0.0;
float diff = 0.05;
int min_timeout = 60000; //in ms
long lastMsg = 0;
char message[24];

void setup() {
	Serial.begin(115200);
	Serial << endl << endl;
	Homie_setFirmware("NaundorfIT_IR", "1.2");
	Homie_setBrand("NaundorfIT");
	Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
	IRdeviceDriver.advertise("IRreceived");
	IRdeviceDriver.advertise("temperature");
	IRdeviceDriver.advertise("cmd").settable(commandHandler);
	Homie.setup();
	irrecv.enableIRIn();                // Start the IR receiver
	irsend.begin();                     // Setting up the IR sending part
	pinMode(BUILTIN_LED, OUTPUT);       // Initialize the BUILTIN_LED pin as an output
	digitalWrite(BUILTIN_LED, HIGH);    // Turn LED off
}
void setupHandler() { 
}
void loop() { Homie.loop(); }
void loopHandler() {
	now = millis();
	if (now - lastMsg > min_timeout) {
		sensors.requestTemperatures(); // Send the command to get temperatures
		float newTemp = sensors.getTempCByIndex(0);
		lastMsg = now;
		if (checkBound(newTemp, temp, diff)) { 
			temp = newTemp; 
			IRdeviceDriver.setProperty("temperature").send(String(temp));
			Homie.getLogger() << "Temperature: " << temp << " �C" << endl;
		}
	}
	if (irrecv.decode(&results)) {
		sprintf(message, "%X", results.value);
		String longmessage = message;
		sprintf(message, "%X", results.address);
		Homie.getLogger() << "IR received: " << results.decode_type << " "<< message << ":" << longmessage << endl;
		IRdeviceDriver.setProperty("IRreceived").send(longmessage);
		irrecv.resume(); // Receive the next value
	}
}

bool commandHandler(const HomieRange& range, const String& value) {
	Homie.getLogger() << "Command received: " << String(value) << endl;
	String cmd = value;
	if (cmd.startsWith("NEC")) {
		unsigned long code = strtoul(cmd.substring(4, 12).c_str(), NULL, 16);
		irsend.sendNEC(code, 32);
		Homie.getLogger() << "NEC code sent: " << cmd.substring(4, 12) << endl;
		delayMicroseconds(100000);			// Time to allow for sending a sequence of IR codes
		// Crash at 600000 microseconds !!
	}
	if (cmd.startsWith("PAN")) {
		uint32_t codepan = strtoul(cmd.substring(4, 14).c_str(), NULL, 16);
		uint16_t add = 0x4004;
		uint16_t repeat = 1;
		irsend.sendPanasonic(add, codepan);
		sprintf(message, "%X", codepan);
		Homie.getLogger() << "PANASONIC code sent: " << message << endl;
		delayMicroseconds(100000);		// Time to allow for sending a sequence of IR codes
	}
	if (cmd.startsWith("TIME")) {
		Serial.println("Time >>>" + cmd.substring(5, 12) + "<<<");
		min_timeout = strtoul(cmd.substring(5, 12).c_str(), NULL, 10);
	}

	if (cmd.startsWith("DELTA")) {
		Serial.println("Delta >>>" + cmd.substring(6, 10) + "<<<");
		diff = strtoul(cmd.substring(6, 10).c_str(), NULL, 10) / 100.;
	}
	return true;
}


//####################################################################
bool checkBound(float newValue, float prevValue, float maxDiff) {
	//return(true);    //Use this line to switch off checking for changes
	return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}