# LED-Card
LED Card for the 2020-21 bot

## Design Specs

Interfaces with LED buffers for neopixels

Flash different statuses (listed below)


## CAN Bus information

### Status Lights
Red/Blue: Not connected

Green: Connected in Manual mode

Purple: Connected in Autonomous mode

### Planned features
Scrolling colors (flashy)

Digging mode colors

More Statuses

Overall more awesomeness

### Overview / Notes
#### dsPIC33EP512GM604

#### System:
24MHz clock rate for chip


##### PLL:
1:30 Prescaler (800kHz)

1:150 FeedBack (120MHz)

1:2 Postscaler (60MHz)

60MHz Fosc

30MHz Fosc/2


##### Watchdog:

Non-Window Mode

1:128 Prescaler

1:32768 Postscaler

128s Timeout

##### DMA:

###### CH0

Continuous Transfer, No Ping-Pong

Triggered on CAN1RX

16bit datasize

0x7 Transfer Count

###### CH1

Continuous Transfer, No Ping-Pong

Triggered on CAN1TX

16bit datasize

0x7 Transfer Count

#### CAN 1 
500kbps
15 quanta

80% sample

Sync,Prop,Phase,Phase
1,3,8,3

CAN Interrupt enabled

DMA Buffer Size 32

2 Transmit Buffers

Message ID 0x100
SID
Filter 0
Mask 0
Rx RB2

Msg ID 0x7c0
SID
Filter 1
Mask 1
Rx RB3

#### CAN 1 
20kbps
15 quanta

80% sample

Sync,Prop,Phase,Phase
1,3,8,3

DMA Buffer Size 32

1 Transmit Buffer

Message ID 0x100
SID
Filter 0
Mask 0
Rx RB2

Msg ID 0x7c0
SID
Filter 1
Mask 1
Rx RB3