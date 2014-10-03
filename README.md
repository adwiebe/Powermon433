Powermon433
===========

ATmega (Arduino) decoding of EM100B and PowerCost Monitor

Forked from [CapnBry/Powermon433](https://github.com/CapnBry/Powermon433 "CapnBry/Powermon433") to provide a simple serial logger.

Bryan's Receiver Notes
----------------------

Data line from a superheterodyne receiver should in run to pin Digital 8.
I've tried using a superregenerative receiver but the sensitity was too low
to receieve anything. Your results may be better than mine.

To use an RF69 / RFM69W / RFM69HW / RFM69CW / RFM69HCW connect:
    RFM NSS (Slave Select) => Digital 10
    RFM MOSI => Digital 11
    RFM MISO => Digital 12
    RFM SCK => Digital 13
    RFM DIO2 => Digital 8

With the RF69 module, a frequency of 433.845MHz is tuned with a 50khz receive
bandwidth. My module can receive data on this frequency right down to a 1.3Khz
bandwidth so I am pretty sure this is accurate. The AFC and FEI blocks don't
appear to work no matter where I trigger them, so I can't use them to adjust
the frequency. Perhaps they only work on FSK and this is ASK/OOK.

Standard auto LNA gain and 'peak' mode OOK threshold are used which does a good
job of adjusting receiver sensitivity. The module registers are assumed to be
at their default values on startup, so be sure to reset the module if switching
from code that uses packet or FSK mode.
 
Use a 164.398mm wire antenna for quarter wavelength monopole.
