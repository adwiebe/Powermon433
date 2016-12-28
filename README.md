Powermon433
===========

ATmega (Arduino) decoding of Blue Line Innovations
[PowerCost Monitor™](http://www.bluelineinnovations.com/powercost-monitor-2
"PowerCost Monitor™") and Black & Decker
[Power Monitor EM100B](http://servicenet.blackanddecker.com/Products/Detail/EM100B
"Power Monitor EM100B") real time household electricity consumption
data. Forked from
[CapnBry/Powermon433](https://github.com/CapnBry/Powermon433
"CapnBry/Powermon433") to provide a simple serial logger and some
usage instructions.

Tested with:

* Arduino Nano + Geetech superheterodyne receiver

A working PowerCost Monitor™ or Power Monitor EM100B (the meter
transmitter is identical) installation is assumed.

## Quick Start ##

1. Wire up one of the “Tested with” options above, according to the
   receiver notes below. The Arduino + superhet receiver option is
   cheapest (but has a minor fiddle with trimming the wire antenna to
   length)
2. Upload the included sketch, and open the serial monitor or serial
   terminal (38400, 8N1). You should soon see some **# CRC ERR**
   messages scroll by, a couple a minute or so.
3. Pull up Pin 5 to HIGH (5V -> 1K -> D5) (install jumper on my setup)
4. Go out to your meter, and briefly press the button on the monitor
   sensor. The red LED in the battery window should start to flash.
5. You *should* see a **New ID** message in the output,
   followed by a hexadecimal number. The CRC ERR messages should soon
   be replaced with energy (Wh), instantaneous power (W) and outside
   temperature readings. Energy and temperature take a couple of
   minutes to come in, so will start at zero.
6. Remove jumper from Pin 5 (should be tied to GND via 10K pull down)

If you're having difficulties, Bryan's original sketch prints more
diagnostic messages than this one. If you are using a wire antenna,
check it's the right length …

## Stewart's Receiver Notes ##

The RFM69 receiver boards described by Bryan (see below) are lovely,
but:

1. They're 3.3 V logic
2. They use 2 mm pitch headers instead of the “standard” 2.54 mm

If you don't want to use an RFM69, you really want to use a
superheterodyne 433.92 MHz receiver instead of a super-regenerative
receiver board.

(Note: RFM69 code was removed from this fork since the PINS in that code
are being used for LED status indicators)

A superheterodyne board:

* is cheap
* has one decently-sized chip on board (the RX3400-LF is 24-pin SSOP),
  which may be covered in a metal shield
* has a crystal (typically 6.773 MHz for 433.92 MHz RX frequency)
* has no obvious coils or tuning chokes

A super-regenerative board:

* is **very** cheap
* typically only has one small chip (like an 8-pin LM358 op-amp)
* has no crystal
* has a coil and a tuning choke
* doesn't work for this application.

### Superheterodyne wiring ###

These boards often have repeated pads. The one I'm using (marked “3953
A434”) works for me when connected as follows:

    Board   Arduino
	=====   =======
     GND  → GND
  	 DATA → D8
 	 DATA → D8
	 +5V  → 5V
      …
     +5V  → 5V
	 GND  → GND
	 GND  → GND
	 ANT  → Antenna (164.5 mm wire as simple ¼-wave monopole)

The antenna length is critical. I had initially cut mine to 168 mm,
and it didn't work at all.

### Parallax 433 MHz RF Transceiver Wiring ###

The Parallax
[433 MHz RF Transceiver](http://www.parallax.com/product/27982 "433
MHz RF Transceiver") uses a Linx TRM-433-LT transceiver chip, and
comes with a nice stub antenna attached. Wiring is simple:

    Board       Arduino
	=====       =======
     1 GND   → GND
	 2 VIN   → 5V
  	 3 DATA  → D8
     4 TX/RX → 5V
	 5 PDN   → GND
	 6 RSSI  → not connected

Although this board is expensive, it's:

1. relatively easy to find in hobby electronics shops
2. already got a properly trimmed antenna.

If time is money for you, the Parallax might be a good option.

Bryan's Receiver Notes
----------------------

Data line from a superheterodyne receiver should in run to pin
Digital 8.  I've tried using a superregenerative receiver but the
sensitivity was too low to receieve anything. Your results may be
better than mine.

To use an RF69 / RFM69W / RFM69HW / RFM69CW / RFM69HCW connect:

    RFM     Arduino
	===     =======
	NSS  → D10
	MOSI → D11
	MISO → D12
	SCK  → D13
	DIO2 → D8
	ANA  → Antenna (164.5 mm wire as simple ¼-wave monopole)
	GND  → GND
	3.3V → 3V3

With the RF69 module, a frequency of 433.845MHz is tuned with a 50khz
receive bandwidth. My module can receive data on this frequency right
down to a 1.3Khz bandwidth so I am pretty sure this is accurate. The
AFC and FEI blocks don't appear to work no matter where I trigger
them, so I can't use them to adjust the frequency. Perhaps they only
work on FSK and this is ASK/OOK.

Standard auto LNA gain and 'peak' mode OOK threshold are used which
does a good job of adjusting receiver sensitivity. The module
registers are assumed to be at their default values on startup, so be
sure to reset the module if switching from code that uses packet or
FSK mode.
 
Use a 164.398mm wire antenna for quarter wavelength monopole.

## RUNNING AS A DATA LOGGER ##

Included in this package is a not-very-well-thought-out example script
`loop.sh` you can run to log data from your power monitor. Check the
comments for requirements.

### THE ARDUINO RESET ####

Arduino boards reset if a new serial connection is made. It's by
design more than anything else, but is annoying if you're making many
short logging runs. In theory, for an Arduino Uno, you can put a 10 µF
capacitor across, but I haven't had much success with that.

As I'm only logging a file once a day, it's not so much of a problem
for me.

## ADDITIONAL NOTES ##

The temperature decoding may not exactly match the display, mainly
because we're using a simple lookup table and interpolating
values. The temperature sensor can be inaccurate anyway, as it's in a
black box often in full sun, and it will often read +10°C over
ambient.

## BUGS ##

1. Yes, I'm sure there are.

## TODO ##

1. Uninitialized values output 0 rather than “NaN” or something more
   appropriate.
