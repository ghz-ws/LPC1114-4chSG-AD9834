import serial
import time
inst=serial.Serial("COM35",9600)
Freq1=[1000000,1000000,1000,1000]  #Hz unit
Pha=[0,0,180,270]               ##deg. unit
Ampl=[2000,2000,2000,2000]  ##mVpp unit. 50ohm loaded.
Freq2=[1700000,1700000]  #kHz unit
Att=[0,0]  ##internal att value. PLL output is 4.65dBm

buf='S1,'+f'{Freq1[0]}'+','+f'{Pha[0]}'+','+f'{Ampl[0]}'+','
inst.write(buf.encode())
buf='S2,'+f'{Freq1[1]}'+','+f'{Pha[1]}'+','+f'{Ampl[1]}'+','
inst.write(buf.encode())
buf='S3,'+f'{Freq1[2]}'+','+f'{Pha[2]}'+','+f'{Ampl[2]}'+','
inst.write(buf.encode())
buf='S4,'+f'{Freq1[3]}'+','+f'{Pha[3]}'+','+f'{Ampl[3]}'+','
inst.write(buf.encode())
buf='S5,'+f'{Freq2[0]}'+','+f'{Att[0]}'+',,'
inst.write(buf.encode())
buf='S6,'+f'{Freq2[1]}'+','+f'{Att[1]}'+',,'
inst.write(buf.encode())

time.sleep(0.2)
buf='?'
inst.write(buf.encode())
buf=inst.read(16)
print(buf)
