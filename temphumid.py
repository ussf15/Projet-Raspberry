#dahi youssef the comments are in english 
import sys
import RPi.GPIO as GPIO
import os
import Adafruit_DHT
import urllib2
import smbus
#import bluetooth
import time
from ctypes import c_short

#Register Address
regCall   = 0xAA
regMean   = 0xF4
regMSB    = 0xF6
regLSB    = 0xF7
regPres   = 0x34
regTemp   = 0x2e

DEBUG = 1
sample = 2
deviceAdd =0x77

humi=""
temp=""

#bus = smbus.SMBus(0)  #for Pi1 uses 0
I2cbus = smbus.SMBus(1) # for Pi2 uses 1

DHTpin = 17

key="ROATBKSGORKOWD3W"       # API key from ThingSpeak

GPIO.setmode(GPIO.BCM)
# Define GPIO to LCD mapping
LCD_RS = 26
LCD_E  = 19
LCD_D4 = 13 
LCD_D5 = 6
LCD_D6 = 5
LCD_D7 = 11

# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

E_PULSE = 0.00005
E_DELAY = 0.00005

#i used this to test with bluetooth 
'''server_socket=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
port = 1
server_socket.bind(("",port))
server_socket.listen(1)
client_socket,address = server_socket.accept()
print "Accepted connection from ",address'''

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)     
GPIO.setup(LCD_E, GPIO.OUT)  
GPIO.setup(LCD_RS, GPIO.OUT) 
GPIO.setup(LCD_D4, GPIO.OUT) 
GPIO.setup(LCD_D5, GPIO.OUT) 
GPIO.setup(LCD_D6, GPIO.OUT) 
GPIO.setup(LCD_D7, GPIO.OUT)
def lcd_init():
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
  GPIO.setup(LCD_E, GPIO.OUT)  # E
  GPIO.setup(LCD_RS, GPIO.OUT) # RS
  GPIO.setup(LCD_D4, GPIO.OUT) # DB4
  GPIO.setup(LCD_D5, GPIO.OUT) # DB5
  GPIO.setup(LCD_D6, GPIO.OUT) # DB6
  GPIO.setup(LCD_D7, GPIO.OUT) # DB7
  # Initialise display
  lcd_byte(0x33,LCD_CMD)
  lcd_byte(0x32,LCD_CMD)
  lcd_byte(0x28,LCD_CMD)
  lcd_byte(0x0C,LCD_CMD)  
  lcd_byte(0x06,LCD_CMD)
  lcd_byte(0x01,LCD_CMD)
  
  

def lcd_string(message,style):
  # Send string to display
  # style=1 Left justified
  # style=2 Centred
  # style=3 Right justified

  if style==1:
    message = message.ljust(LCD_WIDTH," ")  
  elif style==2:
    message = message.center(LCD_WIDTH," ")
  elif style==3:
    message = message.rjust(LCD_WIDTH," ")

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  time.sleep(E_DELAY)    
  GPIO.output(LCD_E, True)  
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)  
  time.sleep(E_DELAY)      

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  time.sleep(E_DELAY)    
  GPIO.output(LCD_E, True)  
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)  
  time.sleep(E_DELAY) 

def convert1(data, i):   # signed 16-bit value
  return c_short((data[i]<< 8) + data[i + 1]).value
 
def convert2(data, i):   # unsigned 16-bit value
  return (data[i]<< 8) + data[i+1] 
   
def readBmp180(addr=deviceAdd):    
  value = bus.read_i2c_block_data(addr, regCall, 22)  # Read calibration data

  # Convert byte data to word values
  AC1 = convert1(value, 0)
  AC2 = convert1(value, 2)
  AC3 = convert1(value, 4)
  AC4 = convert2(value, 6)
  AC5 = convert2(value, 8)
  AC6 = convert2(value, 10)
  B1  = convert1(value, 12)
  B2  = convert1(value, 14)
  MB  = convert1(value, 16)
  MC  = convert1(value, 18)
  MD  = convert1(value, 20)

    # Read temperature
  bus.write_byte_data(addr, regMean, regTemp)
  time.sleep(0.005)
  (msb, lsb) = bus.read_i2c_block_data(addr, regMSB, 2)
  P2 = (msb << 8) + lsb
 
  # Read pressure
  bus.write_byte_data(addr, regMean, regPres + (sample << 6))
  time.sleep(0.05)
  (msb, lsb, xsb) = bus.read_i2c_block_data(addr, regMSB, 3)
  P1 = ((msb << 16) + (lsb << 8) + xsb) >> (8 - sample)

   # Refine temperature
  X1 = ((P2 - AC6) * AC5) >> 15
  X2 = (MC << 11) / (X1 + MD)
  B5 = X1 + X2
  temperature = (B5 + 8) >> 4
 
  # Refine pressure
  B6  = B5 - 4000
  B62 = B6 * B6 >> 12
  X1  = (B2 * B62) >> 11
  X2  = AC2 * B6 >> 11
  X3  = X1 + X2
  B3  = (((AC1 * 4 + X3) << sample) + 2) >> 2
 
  X1 = AC3 * B6 >> 13
  X2 = (B1 * B62) >> 16
  X3 = ((X1 + X2) + 2) >> 2
  B4 = (AC4 * (X3 + 32768)) >> 15
  B7 = (P1 - B3) * (50000 >> sample)
 
  P = (B7 * 2) / B4
 
  X1 = (P >> 8) * (P >> 8)
  X1 = (X1 * 3038) >> 16
  X2 = (-7357 * P) >> 16
  pressure = P + ((X1 + X2 + 3791) >> 4)
  
  return (str(pressure/100.0))

def readDHT():
    humi, temp = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, DHTpin)
    return (str(int(humi)), str(int(temp)))


    
# main() function
def main():
    
    print 'System Ready...'
    URL = 'https://api.thingspeak.com/update?api_key=%s' % key
    print "Wait...."
    while True:
            lcd_init() # lcd init
            (humi, temp)= readDHT() # read temp and humid 
            lcd_byte(LCD_LINE_1, LCD_CMD)
            lcd_string("Humi#   STemp#",1) # show in terminal temp and humid
            lcd_byte(LCD_LINE_2, LCD_CMD)
            lcd_string(humi+'%'+"     %sC " %temp,1)
            finalURL = URL +"&field1=%s&field2=%s"%(humi, temp) 
            print finalURL
            s=urllib2.urlopen(finalURL);# send data to thingspeak
            print  humi+ " " + temp
            #client_socket.send(humi+'%'+"     %sC" %temp)
            s.close()
            finalURL = "http://taguiamine.esy.es/Youssef/savedata.php?temp=%s&humid=%s"%(humi, temp)
            print finalURL
            s=urllib2.urlopen(finalURL); #send data to my server
            s.close()
            time.sleep(2)
            

     
if __name__=="__main__":
   main()