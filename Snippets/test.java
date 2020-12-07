import jpigpio.*;
import rf24j.RF24;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class Test_RF24 {
    private RF24 rf24;
    int cePin = 22;  //GPIO number, e.g. GPIO 22 = PIN 15
    int csnPin = 8;  //GPIO number, e.g. GPIO 8 = PIN 24

    public static void main(String args[]) throws PigpioException {
        System.out.println("Test_RF24");
        Test_RF24 app = new Test_RF24();
        app.run();
    }

    public void run() throws PigpioException {

        System.out.println("Creating pigpio...");
        JPigpio pigpio = new PigpioSocket("pigpiod-host", 8888);

        //JPigpio pigpio = new Pigpio();

        System.out.println("Going to initialize pigpio...");
        pigpio.gpioInitialize();

        System.out.println("Creating RF24...");
        rf24 = new RF24(pigpio);
        p("Initializing...");
        if (!rf24.init(cePin, csnPin)) {
            p("Failed to initialize nRF module. Module not present?");
            rf24.terminate();
            pigpio.gpioTerminate();
            return;
        }

        // 5 byte address width
        rf24.setAddressWidth(5);

        // set remote device address - to which data will be sent and from which data will be received
        byte rcvAddr[] = { '0', 'K', 'E', 'V', 'N' };
        // set transmitter device address - from which data will be sent
        byte sndAddr[] = { '1', 'K', 'E', 'V', 'N' };

        // following params should be configured the same as the other side
        //rf24.setPayloadSize(32); 				// 32 bytes payload
        //rf24.setChannel(76);     				// RF channel
        //rf24.setRetries(5,15);   				// 5 retries, 15x250ms delay between retries
        //rf24.setCRCLength(2);      				// 16 bit CRC
        //rf24.setDataRate(RF24.RF24_1MBPS);	// 1Mbit/s data rate
        //rf24.setAutoACK(false);					// expecting automatic acknowledgements from receiver
        //rf24.setPALevel(RF24.RF24_PA_LOW);  // low power - testing devices won't be so far apart

        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));

		while(1)
		{
        if (rf24.available()) {
			rf24.startListening();
        rf24.read(data32);
		}
		}

        rf24.terminate();
        pigpio.gpioTerminate();

        System.out.println("Done.");

    }

    private void p(String text) {
        System.out.println(text);
    }

    private void logStatus() throws PigpioException {
        byte status = rf24.readByteRegister(RF24.STATUS_REGISTER);
        p(String.format("status = 0x%x %s", status, rf24.statusToString(status)));
    }

    private void logFifoStatus() throws PigpioException {
        byte status = rf24.readByteRegister(RF24.FIFO_STATUS_REGISTER);
        p(String.format("FIFO Status = 0x%x %s", status, rf24.fifoStatusToString(status)));
    }

    private void logConfig() throws PigpioException {
        byte config = rf24.readByteRegister(RF24.CONFIG_REGISTER);
        System.out.println(String.format("config = %x %s", config, rf24.configToString(config)));
    }

    private void showHelp() {
        p("Type:");
        p("s - to switch to sender mode");
        p("r - to switch to receiver mode");
        p("e - to exit");
    }

}