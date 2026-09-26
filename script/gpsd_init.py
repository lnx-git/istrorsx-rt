'''
gpsd_init version 01
'''
from datetime import datetime
import sys
import time
import serial
import serial.tools.list_ports

VERIFY_SECONDS = 6    # at the module's 1 Hz this is ~6 RMC sentences
VERIFY_MIN_RMC = 4    # below this the module stopped, slowed down or went quiet


def nmeaCheck(line):
    # Classify one raw line -> (kind, checksum_ok). kind is 'rmc', 'pmtk',
    # 'other' (a sentence PMTK314 was supposed to switch off) or 'junk'.
    try:
        t = line.decode('ascii', 'replace').strip()
    except Exception:
        return ('junk', False)
    if (not t.startswith('$')) or ('*' not in t):
        return ('junk', False)
    body, _, cks = t[1:].partition('*')
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    ok = (cks[:2].upper() == '%02X' % calc)
    if body.startswith('PMTK'):
        return ('pmtk', ok)
    if body[2:5] == 'RMC':
        return ('rmc', ok)
    return ('other', ok)

list = serial.tools.list_ports.comports()
gpsport = ''
#gpsport = '/dev/ttyUSB0'
gpsbaudrate = 4800

for element in list:
    if element[2].find("VID:PID=067B:2303") >= 0 and element[1] == "USB-Serial Controller D":
        print( 'GPS\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2])
        gpsport = element[0]


if gpsport == '':
    print( 'GPS not found.')
    sys.exit(1)

def ReadGPS(ser, count):
    print( 'Reading GPS:' )
    for x in range(0, count):
        bytes = ser.readline() #reads in bytes followed by a newline
        print(bytes)
        if len(bytes) > 0:
            if bytes[0] == '$':
                print( bytes[:-1]) #print to the console

def WriteGPS(ser, data):
    print ('Writing ' + data + ':' )
    ser.flushInput()
    #ser.write(data+'\r'+'\n')
    ser.write(f"{data}\r\n".encode())

#ser = serial.Serial(gpsport, gpsbaudrate, timeout=0)
ser = serial.Serial(gpsport, gpsbaudrate)
ser.flushInput()
ser.flushOutput()
ser.flush()

ReadGPS(ser,5)

def ExpectPMTK(ser, prefix, seconds=3):
    # Wait for one PMTK reply, ignoring the RMC traffic flowing past. Only the
    # last command is followed by an RMC check -- see below.
    old_timeout = ser.timeout
    ser.timeout = 1        # the port is opened without one: a silent module would block forever
    found = None
    t_end = time.time() + seconds
    while (time.time() < t_end) and (found is None):
        line = ser.readline()
        if not line:
            continue
        kind, ok = nmeaCheck(line)
        text = line.decode('ascii', 'replace').strip()
        if kind == 'pmtk':
            print ('  ' + text + ('' if ok else '   <- BAD CHECKSUM'))
            if text.startswith(prefix) and ok:
                found = text
    ser.timeout = old_timeout
    if found is None:
        print ('  (no %s reply within %ds)' % (prefix, seconds))
    return found


errors = []

print ('\nPMTK_API_SET_NMEA_OUTPUT - only RMC sentences...')
WriteGPS(ser, '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29')
ack = ExpectPMTK(ser, '$PMTK001,314')
if (ack is None) or (not ack.startswith('$PMTK001,314,3')):
    errors.append('PMTK314 (RMC-only output) was not acknowledged: %s' % ack)

print ('\nPMTK_SET_Nav Speed threshold - disable...')
WriteGPS(ser, '$PMTK386,0*23')
ack = ExpectPMTK(ser, '$PMTK001,386')
if (ack is None) or (not ack.startswith('$PMTK001,386,3')):
    errors.append('PMTK386 (nav speed threshold off) was not acknowledged: %s' % ack)

print ('\nPMTK_Q_Nav Speed threshold - query...')
WriteGPS(ser, '$PMTK447*35')
reply = ExpectPMTK(ser, '$PMTK527')
if reply is None:
    errors.append('no $PMTK527 reply, nav speed threshold not confirmed')
else:
    threshold = reply.split(',')[1].split('*')[0]
    if float(threshold) != 0.0:
        errors.append('nav speed threshold is %s, expected 0.00 (PMTK386 did not take effect)' % threshold)

# Only now, with every command acknowledged, is it worth watching the position
# stream: this is what the whole script exists for. The module suppresses
# updates below its nav speed threshold -- built for a car, and a robot at
# walking pace never reaches it, so positions stop advancing. Counting distinct
# UTC stamps rather than sentences is the point: the sentences keep coming.
print ('\nVerifying for %ds that positions really advance...' % VERIFY_SECONDS)
old_timeout = ser.timeout
ser.timeout = 1
counts = {'rmc': 0, 'pmtk': 0, 'other': 0, 'junk': 0}
bad_checksum = 0
others = []
rmc_times = set()
rmc_fix = 0
t_end = time.time() + VERIFY_SECONDS
while time.time() < t_end:
    line = ser.readline()
    if not line:
        continue
    kind, ok = nmeaCheck(line)
    counts[kind] += 1
    if not ok:
        bad_checksum += 1
    text = line.decode('ascii', 'replace').strip()
    if kind == 'other' and len(others) < 5:
        others.append(text)
    if kind == 'rmc' and ok:
        f = text.split(',')
        if len(f) > 2 and f[1]:
            rmc_times.add(f[1])
            if f[2] == 'A':
                rmc_fix += 1
ser.timeout = old_timeout
ser.close()

print ('  rmc=%d updates=%d fix=%d other=%d junk=%d bad_checksum=%d'
       % (counts['rmc'], len(rmc_times), rmc_fix, counts['other'],
          counts['junk'], bad_checksum))
if rmc_fix == 0 and counts['rmc'] > 0:
    print ('  note: no valid fix yet (status V) -- fine indoors, the checks below do not need one')

if len(rmc_times) < VERIFY_MIN_RMC:
    errors.append('only %d position update(s) in %ds (%d RMC sentence(s)), expected at least %d'
                  ' -- the module is still withholding updates at standstill'
                  % (len(rmc_times), VERIFY_SECONDS, counts['rmc'], VERIFY_MIN_RMC))
if counts['other'] > 0:
    errors.append('%d non-RMC sentence(s) still coming, PMTK314 did not take effect: %s'
                  % (counts['other'], others))
if bad_checksum > 0:
    errors.append('%d sentence(s) with a bad checksum' % bad_checksum)
if counts['junk'] > 0:
    errors.append('%d unparsable line(s)' % counts['junk'])

if errors:
    print ('\nERROR: GPS did not accept or is not honouring its configuration:')
    for e in errors:
        print ('  - ' + e)
    sys.exit(1)

print ('\nOK: GPS configured, sending valid RMC only, positions advancing at standstill.')
