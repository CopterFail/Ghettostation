##############################################################################
#                                                                            #
#                         OPEN FPV STATION,                                  #
#       HoTT antenna tracker, diversity receiver, ground osd                 #
#                                                                            #
##############################################################################


Used hardware:
 - Graupner MX12 HoTT with Bluetooth mod (I use the HoTT telemetrie protocol, others are possible)
 - TEENSY3.1
 - HC-05 Bluetooth module for the station, configured as master
 - HC-06 Bluetooth module for the MX12 (slave, other modules can be used)
 - ST7735 LCD display (color, 128x160 pixel)
 - 2 x RX5808 with SPI mod
 - 2 x Antenna
 - 2 x Servo
 - 5V voltage regulator
 - minimosd
 - 3 x Button 

Project status:
 - The station can be used, but is experimental
 - Tracking is ok
 - HoTT link is ok
 - Video receiver manual selection, channel scan and selection is ok
 - Diversity seems ok, but has to be tested


Credits:

Sample codes & inspirations from different origins:
- bluetooth in HoTT TX (http://fpv-community.de/showthread.p...l=1#post354274)
- HoTT smartbox/bt discussion 
  (http://fpv-community.de/showthread.php?50949-HoTT-Protokoll-via-Smartbox-oder-BT-Modul)
- ghettostation (https://github.com/KipK/Ghettostation) from KipK's 
- minimosd-extra (http://code.google.com/p/minimosd-extra/)
- many more...

 
