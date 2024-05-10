# This message contains the angle of arrival measurements computed
# from a particular Wifi message. See Wifi.msg for more details

Header header

uint8 ap_id # AP identifier which collected the CSI data, sequential starting from 0, up to (total number of AP's - 1)
uint8[] txmac
uint8 n_tx # number of antennas on the transmitter antenna
uint8 n_rx # number of antennas on the receiver antenna
uint32 seq # packet number

# RSSI of packet
int32[] rssi

# angle of arrival values computed from n_tx antennas, this must be set
float64[] aoa # AoA in the coordinates of wifi radio

# (CURRENTLY UNUSED) angle of arrival values computed at n_rx antennas
# float64[] aod
# float64[] aod_local