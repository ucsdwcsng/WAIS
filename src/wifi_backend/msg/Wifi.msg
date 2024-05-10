# This message accepts data from a WiFi access point for us in 
# robotics applications. The core data point is CSI, the channel
# state information. 

# CSI matrix is n_sub x n_rows x n_cols

Header header

uint8 ap_id # access point collecting the CSI
uint8[] txmac

uint8 chan # channel number

uint32 n_sub # number of subcarriers
uint8 n_rows # number of rows for the CSI matrix, generally the number of RX antenna
uint8 n_cols # number of columns, generally the number of TX antenna

# bandwidth of the signal, can be 20, 40 or 80 MHz, 
# the number of subcarriers are affected by this numbers
uint8 bw 

# the number of spatial channels. 
# Fewer than n_cols means the columns index greater than number of 
# spatial channels will be zero
uint8 mcs 

int32 rssi # received signal strength across "n_rows" antennas

# uint32 fc # packet type byte
# uint32 seq_num # packet number

float64[] csi_real # channel state information - real part
float64[] csi_imag # channel state information - complex part

string rx_id #flag of receiver

uint32 msg_id #realtime msg for debug
