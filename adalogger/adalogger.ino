#include <stdio.h>
#include <string.h>

#include "src/global.h"
#include "src/rp2040.h"
#include "src/RTC.h"
#include "src/Menu.h"
#include "src/Filing.h"

//-----------------------------------
// implementation
//-----------------------------------

status_t status=CLOSED;
uint32_t loop_count=0;
uint32_t data_count=0;

uint16_t have_disk=0;
void setup() {
  // put your setup code here, to run once:

  // reduce MCU clock
  set_sys_clock_khz(48'000, true);

  neo_pixel_init();
  neo_pixel_show(10, 0, 0);

  while(millis()<(WAIT*1000)) if(Serial) { Serial.print(millis());break;}
  if(Serial) Serial.println("\nAdalogger");

  neo_pixel_show(0, 0, 0);

  for(int p=0;p<30;p++) // disable GIPOs
  { pinMode(p, INPUT); 
    gpio_set_input_enabled(p, false); 
  }

  rtc_setup();

  i2s_setup();
  dma_setup();
  have_disk=SD_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  status = menu(status);
  //
  int32_t *buffer = is2_last_read();
  if(have_disk && buffer)
  {
    if(status != STOPPED)
    {
      status=logger(buffer,status);
      data_count++;
    }
  }
  loop_count++;
  asm("wfi");
}

/********************** end of program ***********************************/

/*
# Circuit python 
# in terminal
# circuitpython_setboard adafruit_feather_rp2040_adalogger
# "c:\program files (x86)\teraterm\ttermpro.exe"

from os import stat, mkdir, chdir, statvfs
from time import time,monotonic,mktime, sleep, struct_time
import gc
import board
import array
import busio
import sdcardio
import storage
import microcontroller
import rtc
from supervisor import runtime
from digitalio import DigitalInOut,Direction

from lib import adafruit_ds3231
from lib import I2S

import alarm

#------------------------------------
# acquisition constants
#------------------------------------
t_on = 60   # seconds
t_acq = 3   # minutes (for continuous recording set t_acq > t_rep
t_rep = 5   # minutes
#------------------------------------
# implementation
#------------------------------------
def prep_header(num_channels, sampleRate, bitsPerSample):
    global header
    header[:4] =  bytes("RIFF", "ascii")  # (4byte) Marks file as RIFF
    header[4:8] = (512-2*4).to_bytes(4, "little" )  # (4byte) File size in bytes
                                                                    # excluding this and RIFF marker
    header[8:12] = bytes("WAVE", "ascii")  # (4byte) File type
    header[12:16] = bytes("fmt ", "ascii")  # (4byte) Format Chunk Marker
    header[16:20] = (16).to_bytes(4, "little")  # (4byte) Length of above format data
    header[20:22] = (1).to_bytes(2, "little")  # (2byte) Format type (1 - PCM)
    header[22:24] = num_channels.to_bytes(2, "little")  # (2byte)
    header[24:28] =  sampleRate.to_bytes(4, "little")  # (4byte)
    header[28:32] =  (sampleRate * num_channels * bitsPerSample // 8).to_bytes(4, "little")  # (4byte)
    header[32:34] =  (num_channels * bitsPerSample // 8).to_bytes(2, "little")  # (2byte)
    header[34:36] =  bitsPerSample.to_bytes(2, "little")  # (2byte)
    header[36:40] =  bytes("JUNK", "ascii")  # (4byte) Junk Chunk Marker
    header[40:44] =  (512-13*4).to_bytes(4, "little")  # (4byte) Junk size in bytes
    header[504:508] =  bytes("data", "ascii")  # (4byte) Data Chunk Marker
    header[508:512] =  (0).to_bytes(4, "little")  # (4byte) Data size in bytes

def update_header(nbytes):
    global header
    header[4:8] = (nbytes + 512 - 2 * 4).to_bytes(4, "little")
    header[508:512] = nbytes.to_bytes(4, "little")

#----------------------------------------------------------
def does_file_exist(filename):
    try:
        status = stat(filename)
        file_exists = True
    except OSError:
        file_exists = False
    return file_exists

def logger(data):
    global status, loop_count, data_count, total_bytes_written
    global old_time, old_hour
    global t_on,t_acq,t_rep
    global led
    global wav
    global header

    if status == CLOSED:
        # open new file
        led.value = True
        t=r.datetime
        date_str =f"{t.tm_year:04d}{t.tm_mon:02d}{t.tm_mday:02d}"
        time_str =f"{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}"
        if t.tm_hour != old_hour:
            Dir0_str = f"/sd/{uid_str}_{date_str}"
            if not does_file_exist(Dir0_str):
                mkdir(Dir0_str)
                print('mkday: ',Dir0_str)
            chdir(Dir0_str)
            #
            Dir1_str = f"{t.tm_hour:02d}"
            if not does_file_exist(Dir1_str):
                print('mkdir: ',Dir1_str)
                mkdir(Dir1_str)
            chdir(Dir1_str)
            old_hour=t.tm_hour
        #
        fname=f"{uid_str}_{date_str}_{time_str}.wav"
        t1=monotonic()
        wav = open(fname, "wb")
        t1=monotonic()-t1
        pos = wav.seek(512)  # position to first byte of Data section in WAV file
        total_bytes_written = 0
        if have_serial==1:
            print('opening:',fname,t1,end=' ')
        status = RECORDING
        led.value = False

    if (status == RECORDING) | (status == MUST_STOP):
        # write data
        led.value = True
        num_bytes_written = wav.write(data)
        total_bytes_written += num_bytes_written
        led.value = False

        # check to close
        tmp_time=(int(time()) % t_on )
        if (tmp_time < old_time) | (status == MUST_STOP):
            # create header for WAV file and write to SD card
            update_header(total_bytes_written)
            wav_header=header
            _ = wav.seek(0)  # return to first byte of Header section in WAV file
            num_bytes_written = wav.write(wav_header)

            # close file
            t1 = monotonic()
            wav.close()
            t1=monotonic()-t1
            #
            num_samples = total_bytes_written // (4 * NCH)
            gc.collect()
            if have_serial==1:
                print('\tnsamp',num_samples, num_samples/fsamp, data_count, loop_count,
                      t1, gc.mem_free(),'\t',hex(data[0]))
            data_count = 0
            loop_count = 0

            # should we stop or do we continue with next file?
            if status==MUST_STOP:
                status=STOPPED
            else:
                status = CLOSED
                hibernate(t_acq,t_rep)
        old_time = tmp_time
    return status

def update_time():
    global r, ext_rtc
    # check RTC clocks
    rd=r.datetime
    ldatestr=f"{rd.tm_mday:02d}-{rd.tm_mon:02d}-{rd.tm_year:04d}"
    ltimestr=f"{rd.tm_hour:02d}:{rd.tm_min:02d}:{rd.tm_sec:02d}"
    print(' local: ',ldatestr,' ',ltimestr)

    dt= ext_rtc.datetime
    datestr=f"{dt.tm_mday:02d}-{dt.tm_mon:02d}-{dt.tm_year:04d}"
    timestr=f"{dt.tm_hour:02d}:{dt.tm_min:02d}:{dt.tm_sec:02d}"
    print('ds3231: ', datestr,' ',timestr)
    print('Is time of ds3231 correct? If yes press return, otherwise')
    strx=input('enter time (dd-mm-yyyy HH:MM:SS): ')
    print(strx)
    if len(strx)>10:
        datestr, timestr = strx.split()
        day,month,year = datestr.split('-')
        hour,minute,second = timestr.split(':')
        print(year,month,day,hour,minute,second)
        tarray=[int(year),int(month),int(day),int(hour),int(minute),int(second),2,-1,-1]
        td=struct_time(tarray)
        datetime = mktime(td)
        print(datetime)
        ext_rtc.datetime = td
    # synchronize rtc
    r.datetime=ext_rtc.datetime

def hibernate(t_acq,t_rep):
    global ext_rtc
    # set ext_rtc alarm
    t1=ext_rtc.datetime
    tmin=t1.tm_min+(t1.tm_sec+2)//60
    #print(t1.tm_min,t1.tm_sec, tmin, tmin%t_rep, t_acq, t_rep)
    if tmin%t_rep<t_acq:
        return
    #
    tmin= ((tmin//t_rep+1)*t_rep)%60  #trigger every 'dmin' minutes
    t2=struct_time([0,0,0,0,tmin,0,-1,-1,-1])
    ext_rtc.alarm1=[t2,'hourly']
    ext_rtc.alarm1_interrupt=True
    ext_rtc.alarm1_status=False
    pin_alarm = alarm.pin.PinAlarm(pin=board.A2, value=False, edge=True, pull=True)
    alarm.exit_and_deep_sleep_until_alarms(pin_alarm)

def menu():
    global have_serial, status
    if have_serial==0:
        if runtime.serial_connected:
            have_serial=1
    if have_serial==1:
        if runtime.serial_bytes_available:
            ch = input().strip()
            if ch == 's':
                status=CLOSED
            elif ch == 'e':
                status=MUST_STOP
            elif (ch == 'c') & (status == STOPPED):
                update_time()
            else:
                print(len(ch),ch)

def wait_for_Serial(secs):
    t0=monotonic()
    while (monotonic()-t0) < secs:
        if runtime.serial_connected:
            sleep(0.1)
            print(monotonic()-t0)
            return 1
    return 0

#====================== Setup ====================================
# general constants not to be changed frequently
SerWait = 3 # seconds to wait for Serial port (interactive)

NCH = 1
fsamp = 48000
microcontroller.cpu.frequency=48_000_000

#-----------------------------------------------------------------
CLOSED      = 0
RECORDING   = 1
MUST_STOP   = 2
STOPPED     = 3
status = STOPPED

loop_count = 0
data_count = 0

header=bytearray(512)
prep_header(num_channels=NCH,sampleRate=fsamp,bitsPerSample=32)

old_time:int = 0
old_hour:int = 24

wav: None
total_bytes_written: int = 0

# Connect to the card and mount the filesystem.
spi = busio.SPI(board.SD_CLK, board.SD_MOSI, board.SD_MISO)
sdcard = sdcardio.SDCard(spi, board.SD_CS, baudrate=24000000)
vfs = storage.VfsFat(sdcard)

storage.mount(vfs, "/sd")
chdir('/sd')

# connect to external RTC
i2c = board.I2C()  # uses board.SCL and board.SDA
ext_rtc = adafruit_ds3231.DS3231(i2c)

r = rtc.RTC()

have_serial=wait_for_Serial(SerWait)
if have_serial>0:
    print('\n**************microPAM********************\n')
    #
    tx= ext_rtc.datetime
    datestr=f"{tx.tm_mday:02d}-{tx.tm_mon:02d}-{tx.tm_year:04d}"
    timestr=f"{tx.tm_hour:02d}:{tx.tm_min:02d}:{tx.tm_sec:02d}"
    print('Now: ', datestr,' ',timestr)
    #
    fs_stat = statvfs('/sd')
    print("Disk size in MB",  fs_stat[0] * fs_stat[2] / 1024 / 1024)
    print("Free space in MB", fs_stat[0] * fs_stat[3] / 1024 / 1024)
    print(spi.frequency)
    print()
    #
else:
    status=CLOSED

#
#===========================================================================================
# synchronize rtc
r.datetime=ext_rtc.datetime
#
uid=microcontroller.cpu.uid
uid_str=f"{uid[-3]:02X}{uid[-2]:02X}{uid[-1]:02X}"
#

i2s = I2S.i2s_ICS43434(fs=fsamp, nbits=32, in_pin=board.D11, bclk_pin=board.D9)

if have_serial>0:
    print()
    print(f"# actual sample frequency {i2s.frequency / 2 / 2 / 32:9.1f} Hz")
    print(f"#               bit clock {i2s.frequency / 2:9.1f} Hz")
    print(f"#             Temperature {microcontroller.cpu.temperature:3.1f}")
    print()

# led is used to indicate disk activity (lights up during file opening)
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT
#

# allocate ping-pong buffers for I2S
NSAMP= 9600
buffer_in1 = array.array("l", (1 for _ in range(NSAMP)))
buffer_in2 = array.array("l", (2 for _ in range(NSAMP)))

# start I2S use ping-pong buffer
i2s.background_read(loop=buffer_in1, loop2=buffer_in2)

def loop():
    global status,data_count,loop_count
    # main loop
    status=CLOSED
    while True:
        menu()
        #
        buffer = i2s.last_read
        if len(buffer) > 0:
            if status != STOPPED:
                logger(buffer)
                data_count += 1
        loop_count += 1
#
loop()
# end of program
*/
