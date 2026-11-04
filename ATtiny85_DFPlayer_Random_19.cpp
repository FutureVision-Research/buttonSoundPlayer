/*
 * ATtiny85 Random DFPlayer Controller
 *
 * Clock frequency: 8 MHz
 *
 * Operation:
 * 1. The DFPlayer Mini and ATtiny85 remain continuously powered.
 * 2. A momentary pushbutton connects PB3 to ground.
 * 3. The internal pullup keeps PB3 high while the button is released.
 * 4. Each button press plays one file from a shuffled list of 19 files.
 * 5. Every file plays once before the list is shuffled again.
 * 6. ADC1 on PB2 supplies noise for the random seed.
 *
 * Files may be stored as:
 *
 *     /MP3/0001.mp3
 *     /MP3/0002.mp3
 *     ...
 *     /MP3/0019.mp3
 *
 * Command 0x03 plays files by the order recognized by the DFPlayer.
 * Copying the files to a freshly formatted card in numerical order helps
 * keep the recognized order consistent with the filenames.
 *
 * ATtiny85 connections:
 *
 *     PB3, physical pin 2: Pushbutton input
 *     PB4, physical pin 3: Serial output to DFPlayer RX through 1 kΩ
 *     PB2, physical pin 7: Floating ADC1 random seed input
 *
 * Pushbutton connection:
 *
 *     PB3 ---- momentary pushbutton ---- GND
 *
 * DFPlayer serial connection:
 *
 *     PB4 ---- 1 kΩ ---- DFPlayer RX
 *
 * The ATtiny85 and DFPlayer must share a common ground.
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define BUTTON_PIN       PB3
#define DFPLAYER_TX_PIN  PB4

#define NUMBER_OF_FILES  19
#define DFPLAYER_VOLUME  20

/*
 * The theoretical period at 9600 baud is 104.17 microseconds.
 * A 100 microsecond delay compensates for the instructions executed
 * between delays. This is the timing verified with this circuit.
 */
#define SERIAL_BIT_TIME_US 100

/* State for the 16 bit pseudorandom number generator. */
static uint16_t randomState = 1;

/* Shuffled list containing every available file number. */
static uint8_t shuffledFiles[NUMBER_OF_FILES];

/* Starting at the end forces a shuffle before the first selection. */
static uint8_t nextShuffledFile = NUMBER_OF_FILES;

/* Prevents a repeat across the boundary between two shuffled lists. */
static uint8_t previousFile = 0;

/*
 * Send one byte to the DFPlayer at approximately 9600 baud, 8 data bits,
 * no parity, and one stop bit.
 */
static void softwareSerialWrite(uint8_t value)
{
    uint8_t savedStatusRegister = SREG;
    cli();

    /* Start bit. */
    PORTB &= ~(1 << DFPLAYER_TX_PIN);
    _delay_us(SERIAL_BIT_TIME_US);

    /* Eight data bits, least significant bit first. */
    for (uint8_t bitNumber = 0; bitNumber < 8; bitNumber++)
    {
        if (value & 0x01)
        {
            PORTB |= (1 << DFPLAYER_TX_PIN);
        }
        else
        {
            PORTB &= ~(1 << DFPLAYER_TX_PIN);
        }

        _delay_us(SERIAL_BIT_TIME_US);
        value >>= 1;
    }

    /* Stop bit and idle state. */
    PORTB |= (1 << DFPLAYER_TX_PIN);
    _delay_us(SERIAL_BIT_TIME_US);

    SREG = savedStatusRegister;
}

/* Send one complete ten byte command packet to the DFPlayer. */
static void sendDFPlayerCommand(uint8_t command, uint16_t parameter)
{
    uint8_t parameterHigh = (uint8_t)(parameter >> 8);
    uint8_t parameterLow  = (uint8_t)(parameter & 0xFF);

    uint16_t checksum =
        0 - (0xFF + 0x06 + command + 0x00 +
             parameterHigh + parameterLow);

    uint8_t packet[10];

    packet[0] = 0x7E;
    packet[1] = 0xFF;
    packet[2] = 0x06;
    packet[3] = command;
    packet[4] = 0x00;
    packet[5] = parameterHigh;
    packet[6] = parameterLow;
    packet[7] = (uint8_t)(checksum >> 8);
    packet[8] = (uint8_t)(checksum & 0xFF);
    packet[9] = 0xEF;

    for (uint8_t index = 0; index < 10; index++)
    {
        softwareSerialWrite(packet[index]);
    }
}

/* Read the floating ADC1 input on PB2. */
static uint16_t readADC1(void)
{
    /* Use VCC as the reference and select ADC1. */
    ADMUX = (1 << MUX0);

    /* Disable the digital input buffer on ADC1. */
    DIDR0 |= (1 << ADC1D);

    /* Enable the ADC with a divide by 64 prescaler: 8 MHz / 64 = 125 kHz. */
    ADCSRA =
        (1 << ADEN)  |
        (1 << ADPS2) |
        (1 << ADPS1);

    /* Perform and discard the first conversion after enabling the ADC. */
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
    {
    }

    /* Perform the conversion used for the seed. */
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
    {
    }

    uint16_t adcReading = ADC;

    /* Disable the ADC after the reading. */
    ADCSRA &= ~(1 << ADEN);

    return adcReading;
}

/* Combine several floating ADC1 readings into the initial random state. */
static void initializeRandomGenerator(void)
{
    uint16_t seed = readADC1();

    _delay_us(37);
    seed ^= (uint16_t)(readADC1() << 3);

    _delay_us(53);
    seed ^= (uint16_t)(readADC1() << 7);

    _delay_us(71);
    seed ^= (uint16_t)(readADC1() << 11);

    /* A xorshift generator must not start with zero. */
    if (seed == 0)
    {
        seed = 0xA5C3;
    }

    randomState = seed;
}

/* Generate the next 16 bit pseudorandom value. */
static uint16_t nextRandomNumber(void)
{
    randomState ^= randomState << 7;
    randomState ^= randomState >> 9;
    randomState ^= randomState << 8;

    return randomState;
}

/* Fill the list with file numbers 1 through 19 and shuffle them. */
static void shuffleFileList(void)
{
    for (uint8_t index = 0; index < NUMBER_OF_FILES; index++)
    {
        shuffledFiles[index] = index + 1;
    }

    /* Fisher Yates shuffle. */
    for (uint8_t index = NUMBER_OF_FILES - 1; index > 0; index--)
    {
        uint8_t randomIndex =
            (uint8_t)(nextRandomNumber() % (index + 1));

        uint8_t temporaryFile = shuffledFiles[index];
        shuffledFiles[index] = shuffledFiles[randomIndex];
        shuffledFiles[randomIndex] = temporaryFile;
    }

    /* Do not repeat the previous file at the start of a new shuffle. */
    if ((NUMBER_OF_FILES > 1) &&
        (shuffledFiles[0] == previousFile))
    {
        uint8_t temporaryFile = shuffledFiles[0];
        shuffledFiles[0] = shuffledFiles[1];
        shuffledFiles[1] = temporaryFile;
    }

    nextShuffledFile = 0;
}

/* Return the next file number from the shuffled list. */
static uint8_t selectRandomFile(void)
{
    if (nextShuffledFile >= NUMBER_OF_FILES)
    {
        shuffleFileList();
    }

    uint8_t selectedFile = shuffledFiles[nextShuffledFile];
    nextShuffledFile++;
    previousFile = selectedFile;

    return selectedFile;
}

/* Play a file by its index on the microSD card. */
static void playFile(uint8_t fileNumber)
{
    sendDFPlayerCommand(0x03, fileNumber);
}

/* Return true while the active low button is pressed. */
static bool buttonPressed(void)
{
    return (PINB & (1 << BUTTON_PIN)) == 0;
}

int main(void)
{
    /* Configure PB4 as the serial output and set its idle state high. */
    DDRB |= (1 << DFPLAYER_TX_PIN);
    PORTB |= (1 << DFPLAYER_TX_PIN);

    /* Configure PB3 as an input and enable its internal pullup. */
    DDRB &= ~(1 << BUTTON_PIN);
    PORTB |= (1 << BUTTON_PIN);

    /* Seed the random number generator from floating ADC1 noise. */
    initializeRandomGenerator();

    /* Select the microSD card as the DFPlayer playback source. */
    sendDFPlayerCommand(0x09, 0x0002);

    /* Allow the source selection command to complete. */
    _delay_ms(500);

    /* Set the volume to 20 out of 30. */
    sendDFPlayerCommand(0x06, DFPLAYER_VOLUME);
    _delay_ms(200);

    while (1)
    {
        if (buttonPressed())
        {
            /* Debounce the button press. */
            _delay_ms(20);

            if (buttonPressed())
            {
                uint8_t selectedFile = selectRandomFile();
                playFile(selectedFile);

                /* Wait for release so one press sends only one command. */
                while (buttonPressed())
                {
                }

                /* Debounce the button release. */
                _delay_ms(20);
            }
        }
    }

    return 0;
}
