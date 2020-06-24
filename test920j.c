#include "test920j_ide.h"		// Additional Header
/* FILE NAME: test920j.c
 * The MIT License (MIT)
 * 
 * Copyright (c) 2017  Lapis Semiconductor Co.,Ltd.
 * All rights reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
*/
#include "driver_gpio.h"
#include "lp_manage.h"
#include "phy/phy_ml7396.h"
#include "hwif/hal.h"
#include "hwif/wire0.h"
#include "hwif/spi0.h"

#include <string.h>

#define DEBUG_SERIAL
static uint8_t pinSleep=0;
static uint8_t baseChannel=24;
static bool nSleep=true;
static uint8_t pinRecv=0;
static char cmdbuf[260];
static char wbuf[256];
static char obuf[230];
static uint16_t wlen=0;
static uint16_t windex=0;

static char rbuf[256];
static uint16_t rlen=0;
static uint16_t rindex=0;
static bool bRemote=false;
static bool sgRxEvent=false;
static bool sgRxAuto = false;

#define CMD_INPUT "i"
#define CMD_OUTPUT "o"
#define CMD_INPUT_PULLDOWN "pd"
#define CMD_INPUT_PULLUP "pu"
#define CMD_HIZ "hz"
#define CMD_DIGITAL_WRITE "dw"
#define CMD_DIGITAL_READ "dr"
#define CMD_PIN_MODE "pm"

#define CMD_SUBGHZ_INIT "sgi"
#define CMD_SUBGHZ_BEGIN "sgb"
#define CMD_SUBGHZ_SEND "sgs"
#define CMD_SUBGHZ_CONTINUE_SENDING "sgcs"
#define CMD_SUBGHZ_SEND_HOPPING "sghp"
#define CMD_SUBGHZ_RXENABLE "sgre"
#define CMD_SUBGHZ_RX_AUTO "sgra"
#define CMD_SUBGHZ_RXDISABLE "sgrd"
#define CMD_SUBGHZ_SET_SEND_MODE "sgssm"
#define CMD_SUBGHZ_GET_SEND_MODE "sggsm"
#define CMD_SUBGHZ_CLOSE "sgc"
#define CMD_SUBGHZ_READ "sgr"
#define CMD_SUBGHZ_READ_BINARY "sgrb"
#define CMD_SUBGHZ_GET_MY_ADDRESS "sggma"
#define CMD_SUBGHZ_GET_MY_ADDR64 "sggma64"
#define CMD_SUBGHZ_SET_MY_ADDRESS "sgsma"
#define CMD_SUBGHZ_GET_STATUS "sggs"
#define CMD_SUBGHZ_SET_ANTSW "sgansw"
#define CMD_WRITE_DATA "w"
#define CMD_WRITE_BINARY "wb"

#define CMD_WIRE_BEGIN 				"wireb"
#define CMD_WIRE_BEGIN_TRANSMITTION	"wirebt"
#define CMD_WIRE_END_TRANSMITTION	"wireet"
#define CMD_WIRE_WRITE				"wirew"
#define CMD_WIRE_AVAILABLE			"wirea"
#define CMD_WIRE_REQUEST_FROM		"wirerf"
#define CMD_WIRE_READ				"wirer"

#define CMD_SPI_BEGIN				"spib"
#define CMD_SPI_TRANSFER			"spit"
#define CMD_SPI_BIT_ORDER			"spibo"
#define CMD_SPI_DATA_MODE			"spidm"
#define CMD_SPI_CLOCK_DIVIDER		"spicd"
#define CMD_SPI_END					"spie"

#define CMD_BIND_SLEEP_PIN			"bindslp"
#define CMD_BIND_RECV_PIN			"bindrcv"

#define CMD_RESET "rst"
#define CMD_SUBGHZ_REMOTE "sgremote"
#define CMD_SET_BAUD "sb"

#define CMD_FLASH_WR				"fwr"
#define CMD_FLASH_RD				"frd"
#define CMD_FLASH_ERASE				"fer"


#define CMD_MILLIS					"millis"

// *************************** TEST PROGRAM ********************************
#define CMD_DEEP_HALT "dh"
#define CMD_EEPROM_WPB "ewp"
#define CMD_EEPROM_WRITE "ewr"
#define CMD_EEPROM_READ "erd"
#define CMD_RF_WRITE "rfw"
#define CMD_RF_READ "rfr"
// *************************** TEST PROGRAM ********************************

#define PARAM_ADDR_TYPE "at"
#define PARAM_SENSE_TYME "st"
#define PARAM_TX_RETRY "tr"
#define PARAM_TX_INTERVAL "ti"
#define PARAM_CCA_WAIT "cw"

#define TXLED 25
#define RXLED 26


static const uint32_t baud_list[]={
	9600,
	19200,
	28800,
	38400,
	57600,
	115200,
	0
};

static bool extEvent = false;

static bool check_baud(uint32_t baud)
{
	int i=0;
	bool result = false;
	do {
		if(baud_list[i] == baud) {
			result = true;
			break;
		}
		i++;
	} while(baud_list[i]!=0);
	return result;
}

static void gpio_write(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	char* en;
	int portNum;
	int levelNum;
	int i=0;
	
	// command sprit
	do {
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	portNum = (int)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	levelNum = (int)strtol(pparam[2],&en,0);
	if(*en != NULL) return;

	// command process
	digitalWrite(portNum,levelNum);

	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p(CMD_DIGITAL_WRITE);
	Print.p(",");
	Print.l((long)portNum,DEC);
	Print.p(",");
	Print.l((long)levelNum,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
	
	return;
}

static void gpio_read(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	char* en;
	int portNum;
	int levelNum;
	int i=0;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	portNum = (int)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	
	// command process
	levelNum = digitalRead(portNum);

	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p(CMD_DIGITAL_READ);
	Print.p(",");
	Print.l((long)portNum,DEC);
	Print.p(",");
	Print.l((long)levelNum,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
	
	return;

}
static void gpio_set(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	char* en;
	int portNum;
	int i=0;
	bool result=true;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	portNum = (int)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	
	if(strncmp(pparam[2],CMD_INPUT,16)==0) {
		pinMode(portNum,INPUT);
	} else if(strncmp(pparam[2],CMD_OUTPUT,16)==0) {
		pinMode(portNum,OUTPUT);
	} else if(strncmp(pparam[2],CMD_INPUT_PULLDOWN,16)==0) {
		pinMode(portNum,INPUT_PULLDOWN);
	} else if(strncmp(pparam[2],CMD_INPUT_PULLUP,16)==0) {
		pinMode(portNum,INPUT_PULLUP);
	} else if(strncmp(pparam[2],CMD_HIZ,16)==0) {
		pinMode(portNum,HIZ);
	} else {
		return;
	}
	Print.init(obuf,sizeof(obuf));
	Print.p(CMD_PIN_MODE);
	Print.p(",");
	Print.l((long)portNum,DEC);
	Print.p(",");
	Print.p(pparam[2]);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
}

/*
Flash.write(sector,address,data);
Flash.read(sector,address);
Flash.erase(sector);
*/
static void fwr(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	char* en;
	unsigned char sector;
	unsigned short address;
	unsigned short data;
	int i=0;
	bool result=true;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	sector = (unsigned char)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	address = (unsigned short)strtol(pparam[2],&en,0);
	if(*en != NULL) return;
	data = (unsigned short)strtol(pparam[3],&en,0);
	if(*en != NULL) return;
	
	if(sector > 2) return;
	if(address > 0x03FF) return;
	
	Flash.write(sector,address,data);
	
	Print.init(obuf,sizeof(obuf));
	Print.p(CMD_FLASH_WR);
	Print.p(",");
	Print.l((long)sector,DEC);
	Print.p(",");
	Print.p(pparam[1]);
	Print.p(",");
	Print.p(pparam[2]);
	Print.p(",");
	Print.p(pparam[3]);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
}

static void frd(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	char* en;
	unsigned char sector;
	unsigned short address;
	unsigned short data;
	int i=0;
	bool result=true;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	sector = (unsigned char)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	address = (unsigned char)strtol(pparam[2],&en,0);
	if(*en != NULL) return;
	
	if(sector > 2) return;
	if(address > 0x03FF) return;
	
	data=Flash.read(sector,address);
	
	Print.init(obuf,sizeof(obuf));
	Print.p(CMD_FLASH_RD);
	Print.p(",");
	Print.l((long)sector,DEC);
	Print.p(",");
	Print.p(pparam[1]);
	Print.p(",0x");
	Print.l(data,HEX);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
}

static void fer(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	char* en;
	unsigned char sector;
	int i=0;
	bool result=true;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	sector = (unsigned char)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	
	if(sector > 2) return;
	
	Flash.erase(sector);
	
	Print.init(obuf,sizeof(obuf));
	Print.p(CMD_FLASH_ERASE);
	Print.p(",");
	Print.l((long)sector,DEC);
	Print.p(",");
	Print.p(pparam[1]);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
}

static void sgi(uint8_t** pparam){
	SubGHz.init();
	Serial.println("sgi");
}

static void sgb(uint8_t** pparam){
	int i=0;
	char* en;
	uint8_t ch;
	uint16_t panid;
	uint8_t rate;
	uint8_t pwr;
	SUBGHZ_MSG msg;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	ch = (uint8_t)strtol(pparam[1],&en,0);
    baseChannel = ch;
	if(*en != NULL) return;
	panid = (uint16_t)strtol(pparam[2],&en,0);
	if(*en != NULL) return;
	rate = (uint8_t)strtol(pparam[3],&en,0);
	if(*en != NULL) return;
	pwr = (uint8_t)strtol(pparam[4],&en,0);
	if(*en != NULL) return;
	if(
		(((ch>=24)&&(ch<=61)&&(rate==50))||((ch>=24)&&(ch<=60)&&(rate==100))) &&
		((pwr == 1) || (pwr == 20)) 
	)
	{
		msg = SubGHz.begin(ch, panid, rate, pwr);
		Serial.print("sgb,");
		Serial.print_long(ch,DEC);
		Serial.print(",0x");
		Serial.print_long(panid,HEX);
		Serial.print(",");
		Serial.print_long(rate,DEC);
		Serial.print(",");
		Serial.print_long(pwr,DEC);
		Serial.print(",");
		Serial.println_long(msg,DEC);
	}
}

static void sgs(uint8_t** pparam){
	int i=0;
	char* en;
	uint16_t distPanid;
	uint16_t distAddr;
	uint8_t ackReq;
	SUBGHZ_STATUS tx;
	SUBGHZ_MSG msg;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	distPanid = (uint16_t)strtol(pparam[1],&en,0);
	distAddr = (uint16_t)strtol(pparam[2],&en,0);
	ackReq= (uint8_t)strtol(pparam[3],&en,0);
	if(ackReq){
		SubGHz.setAckReq(true);
	}else{
		SubGHz.setAckReq(false);
	}
	digitalWrite(TXLED,LOW);
  	msg = SubGHz.send(distPanid,distAddr,wbuf,wlen,NULL);
	digitalWrite(TXLED,HIGH);

	Serial.print("sgs,");
	Serial.print(pparam[1]);
	Serial.print(",");
	Serial.print(pparam[2]);
	Serial.print(",");
	Serial.print(pparam[3]);
	Serial.print(",");
	Serial.print_long(msg,DEC);
	if(msg==SUBGHZ_OK) 
	{
		Serial.print(",");
		SubGHz.getStatus(&tx,NULL);
		Serial.println_long(tx.rssi,DEC);		
	} else {
		Serial.println("");
	}
}

static void sgcs(uint8_t** pparam){
	int i=0;
	char* en;
	uint16_t distPanid=0xabcd;
	uint16_t distAddr=0xffff;
	uint16_t len;
	uint16_t packets;
	SUBGHZ_STATUS tx;
	SUBGHZ_MSG msg;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	len = (int)strtol(pparam[1],&en,0);
	packets = (int)strtol(pparam[2],&en,0);

    for(i=0; i < len; i++){
        wbuf[i]=i;
    }

	if((len<20)||(len>230)) {
		goto error;
	} else {		
#ifdef DEBUG_SERIAL
		Serial.print("sgcs,");
		Serial.print_long(len,DEC);
		Serial.print(",0x");
		Serial.print_long(packets,HEX);
		Serial.print(",");
	    Serial.println(wbuf);
#endif	
	}

	delay(100);

    for(i=0; i < packets; i++){
        digitalWrite(TXLED,LOW);
        msg = SubGHz.send(distPanid,distAddr,wbuf,len,NULL);
        digitalWrite(TXLED,HIGH);
    }
error:
	return;
}

static void sghp(uint8_t** pparam){
	int i,send_cnt=0;
	char* en;
	uint8_t index=11;
//	uint8_t ch[]={24,26,28,30,33,35,37,39,41,43};
//	uint8_t ch[]={35,24,43,37,26,41,28,39,30,33};
	uint8_t ch[12]={34,24,42,36,26,40,28,38,30,32,44,46};
	uint16_t panid=0xabcd;
	uint16_t distPanid=0xffff;
	uint16_t distAddr=0xffff;
	uint8_t rate=100;
	uint8_t pwr=20;
	uint8_t len=230;
	uint8_t pkt=1;
	uint8_t ch_num=1;
	uint8_t min_ch=baseChannel; // baseChannel was set by sgb command.
	uint8_t max_ch;
	SUBGHZ_MSG msg;
    
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	ch_num = (int)strtol(pparam[1],&en,0);
	pkt = (int)strtol(pparam[2],&en,0);
    max_ch = min_ch + ((ch_num-1)*2);

    if (max_ch >= ch[index]) {
        Serial.println("sghp,invalid parameter");
        goto error;
    }

    for(i=0; i <len ; i++){
        wbuf[i]=i;
    }

    for(send_cnt=0; send_cnt <pkt ; send_cnt++){
        for(i=0; i <index ; i++){
        	if (ch[i] >= min_ch && ch[i] <= max_ch) {
              digitalWrite(TXLED,LOW);
              msg = SubGHz.begin(ch[i], panid, rate, pwr);
              msg = SubGHz.send(distPanid,distAddr,wbuf,len,NULL);
              digitalWrite(TXLED,HIGH);
        	}
        }
    }

    Serial.print("sghp,");
    Serial.print_long(ch_num,DEC);
    Serial.print(",");
    Serial.println_long(pkt,DEC);
error:
    return;
}

static void write_data(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac){
	// command sprit
	pparam[1] = strtok(NULL,"\r\n");
	if(pparam[1]==NULL) return;

	// command process
	strncpy(wbuf,pparam[1],256);
	wlen = strlen(pparam[1]);

	Serial.print("w,");
	Serial.println(wbuf);
}

static void write_binary(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	char* en;
	volatile int rtmp;
	int i=0;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	wlen = (uint16_t)strtol(pparam[1],&en,0);
	windex = 0;
	if(wlen <=230) {

		Serial.print("wb,");
		Serial.println_long(wlen,DEC);

		while(windex<wlen)
		{
			rtmp=Serial.read();
//			Serial.println_long(rtmp,DEC);
			if(rtmp>=0) {
				wbuf[windex] = (uint8_t)rtmp;

				Serial.print_long(rtmp,HEX);
				if(windex%16==15) {
					Serial.println("");
				} else {
					Serial.print(" ");
				}

				windex++;
			}
//			delay(1000);
		}
		Serial.println("");
	}
}

static void rxCallback(uint8_t *data, uint8_t rssi, int status)
{
	if(status > 0)
	{
		if(pinRecv) digitalWrite(pinRecv,HIGH);
		digitalWrite(RXLED,LOW);
		if(bRemote==true)
		{
			sgRxEvent = true;
//			digitalWrite(RXLED,LOW);
		}
	}
}

static void sgra(uint8_t** pparam)
{
	SubGHz.rxEnable(rxCallback);
	sgRxAuto = true;
	Serial.println("sgra");

}

static void sgre(uint8_t** pparam)
{
	SubGHz.rxEnable(rxCallback);
	sgRxAuto = false;

	Serial.println("sgre");

}

static void sgrd(uint8_t** pparam)
{
	SubGHz.rxDisable();
	sgRxAuto = false;

	Serial.println("sgrd");

}

static void sgssm(uint8_t** pparam)
{
	int i=0;
	char* en;
	uint16_t tmp;
	SUBGHZ_PARAM sgParam;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	SubGHz.getSendMode(&sgParam);

	tmp = (uint16_t)strtol(pparam[2],&en,0);

	if(strncmp(pparam[1],PARAM_ADDR_TYPE,16)==0) {
		if((tmp>=0) && (tmp<=7)) {
			sgParam.addrType = (uint8_t)tmp;
			SubGHz.setSendMode(&sgParam);
#ifdef DEBUG_SERIAL
			Serial.print("sgssm,at,");
			Serial.println_long(tmp,DEC);			
#endif
		}
		
	}
	else if(strncmp(pparam[1],PARAM_SENSE_TYME,16)==0) {
			sgParam.senseTime = (uint8_t)tmp;
			SubGHz.setSendMode(&sgParam);
#ifdef DEBUG_SERIAL
			Serial.print("sgssm,st,");
			Serial.println_long(tmp,DEC);			
#endif
	}
	else if(strncmp(pparam[1],PARAM_TX_RETRY,16)==0) {
			sgParam.txRetry = (uint8_t)tmp;
			SubGHz.setSendMode(&sgParam);
#ifdef DEBUG_SERIAL
			Serial.print("sgssm,tr,");
			Serial.println_long(tmp,DEC);			
#endif
	}
	else if(strncmp(pparam[1],PARAM_TX_INTERVAL,16)==0) {
			sgParam.txInterval = (uint16_t)tmp;
			SubGHz.setSendMode(&sgParam);
#ifdef DEBUG_SERIAL
			Serial.print("sgssm,ti,");
			Serial.println_long(tmp,DEC);			
#endif
	}
	else if(strncmp(pparam[1],PARAM_CCA_WAIT,16)==0) {
			sgParam.txInterval = (uint16_t)tmp;
			SubGHz.setSendMode(&sgParam);
#ifdef DEBUG_SERIAL
			Serial.print("sgssm,cw,");
			Serial.println_long(tmp,DEC);			
#endif
	}
	SubGHz.rxDisable();
}
static void sggsm(uint8_t** pparam)
{
	int i=0;
	SUBGHZ_PARAM sgParam;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	SubGHz.getSendMode(&sgParam);

#ifdef DEBUG_SERIAL
	Serial.print("sggsm,");
#endif
	Serial.print_long(sgParam.addrType,DEC);
	Serial.print(",");
	Serial.print_long(sgParam.senseTime,DEC);
	Serial.print(",");
	Serial.print_long(sgParam.txRetry,DEC);
	Serial.print(",");
	Serial.print_long(sgParam.txInterval,DEC);
	Serial.print(",0x");
	Serial.print_long(sgParam.myAddress,HEX);
	Serial.print(",");
	Serial.println_long(sgParam.ccaWait,HEX);
}

static void sgc(uint8_t** pparam)
{
	int i=0;
	SUBGHZ_MSG msg;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	msg=SubGHz.close();
#ifdef DEBUG_SERIAL
	Serial.print("sgc,");
	Serial.println_long(msg,DEC);	
#endif
}

static void sggma(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	uint16_t adr;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	adr = SubGHz.getMyAddress();
#ifdef DEBUG_SERIAL
	Serial.print("sggma,");
#endif
	Serial.print("0x");	
	Serial.println_long(adr,HEX);
}

static void sggma64(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	uint8_t adr[8];
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	SubGHz.getMyAddr64(adr);
#ifdef DEBUG_SERIAL
	Serial.print("sggma64,");
#endif
	for(i=0;i<8;i++) {
		if(adr[i] < 16) {
			Serial.print("0");
		}
		Serial.print_long(adr[i],HEX);
	}
	Serial.println("");
}

static void sgsma(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;
	uint16_t addr;
	SUBGHZ_MSG msg;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	addr = (uint16_t)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	{
		msg = SubGHz.setMyAddress(addr);
		Serial.print("sgsma,0x");
		Serial.print_long(addr,HEX);
		Serial.print(",");
		Serial.println_long(msg,DEC);
	}
}

static void sggs(uint8_t** pparam)
{
	int i=0;
	SUBGHZ_STATUS txStatus;
	SUBGHZ_STATUS rxStatus;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	SubGHz.getStatus(&txStatus,&rxStatus);
#ifdef DEBUG_SERIAL
	Serial.print("sggs,");
#endif
	Serial.print_long(txStatus.rssi,DEC);
	Serial.print(",");
	Serial.print_long(txStatus.status,DEC);
	Serial.print(",");
	Serial.print_long(rxStatus.rssi,DEC);
	Serial.print(",");
	Serial.println_long(rxStatus.status,DEC);
}

static void sgansw(uint8_t** pparam)
{
	int i=0;
	char* en;
	uint8_t ant_sw;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	ant_sw = (unsigned char)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
	{
		SubGHz.antSwitch(ant_sw);
		Serial.print("sgansw,0x");
		Serial.println_long(ant_sw,HEX);
	}
}

static void sgout()
{
	int i=0;
	short result;
	char* en;
	SUBGHZ_MAC_PARAM mac;
	SUBGHZ_STATUS rx;
	uint16_t addr;
	
	// command process
	result = SubGHz.readData(rbuf,sizeof(rbuf));
#ifdef DEBUG_SERIAL
	Serial.print("sgout,");
#endif	
	Serial.print_long(result,DEC);
	Serial.print(",");
	if(pinRecv) digitalWrite(pinRecv,LOW);
	digitalWrite(RXLED,HIGH);
	if(result > 0)
	{
		SubGHz.getStatus(NULL,&rx);
		SubGHz.decMac(&mac,rbuf,result);
/*
		Serial.print("0x");
		Serial.print_long(mac.mac_header.fc16,HEX);
		Serial.print(",0x");
		Serial.print_long(mac.seq_num,HEX);
		Serial.print(",0x");
		Serial.print_long(mac.dst_panid,HEX);
		Serial.print(",0x");
		addr=mac.dst_addr[1];
		addr = (addr << 8) + mac.dst_addr[0];
		Serial.print_long(addr,HEX);
		Serial.print(",0x");
		Serial.print_long(mac.src_panid,HEX);
		Serial.print(",0x");
		addr=mac.src_addr[1];
		addr = (addr << 8) + mac.src_addr[0];
		Serial.print_long(addr,HEX);
		Serial.print(",");
*/
		Serial.write(mac.payload,result);
		Serial.println("");
	}
}

static void sgr(uint8_t** pparam)
{
	int i=0;
	short result;
	char* en;
	SUBGHZ_MAC_PARAM mac;
	SUBGHZ_STATUS rx;
	uint16_t addr;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	result = SubGHz.readData(rbuf,sizeof(rbuf));
#ifdef DEBUG_SERIAL
	Serial.print("sgr,");
#endif	
	Serial.println_long(result,DEC);
	if(pinRecv) digitalWrite(pinRecv,LOW);
	digitalWrite(RXLED,HIGH);
	if(result >= 0)
	{
		SubGHz.getStatus(NULL,&rx);
		SubGHz.decMac(&mac,rbuf,result);
		Serial.print("0x");
		Serial.print_long(mac.mac_header.fc16,HEX);
		Serial.print(",0x");
		Serial.print_long(mac.seq_num,HEX);
		Serial.print(",0x");
		Serial.print_long(mac.dst_panid,HEX);
		Serial.print(",0x");
		addr=mac.dst_addr[1];
		addr = (addr << 8) + mac.dst_addr[0];
		Serial.print_long(addr,HEX);
		Serial.print(",0x");
		Serial.print_long(mac.src_panid,HEX);
		Serial.print(",0x");
		addr=mac.src_addr[1];
		addr = (addr << 8) + mac.src_addr[0];
		Serial.print_long(addr,HEX);
		Serial.print(",");
		Serial.write(mac.payload,result);
		Serial.println("");
	}
}

static void sgrb(uint8_t** pparam)
{
	int i=0;
	short result;
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	result = SubGHz.readData(rbuf,sizeof(rbuf));
	if(pinRecv) digitalWrite(pinRecv,LOW);
	digitalWrite(RXLED,HIGH);
#ifdef DEBUG_SERIAL
	Serial.print("sgrb,");
#endif
	Serial.println_long(result,DEC);
	if(result > 0)
	{
		Serial.write(rbuf,result);
	}
}

static void sb(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;	
	uint32_t baud;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	baud = (uint32_t)strtol(pparam[1],&en,0);	
	if(check_baud(baud))
	{
	 	Flash.erase(0);
		Flash.write(0,0,(unsigned short)((baud>>16)&0x0000FFFF));
		Flash.write(0,1,(unsigned short)(baud&0x0000FFFF));
		Serial.print("sb,");
		Serial.print_long(baud,DEC);
		Serial.println(". Baudrate is changed. Please restart.....");
	}
}
static void sgremote(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	char tmp;
	char* en;
	int i;	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	tmp = (uint32_t)strtol(pparam[1],&en,0);
	if((*en==NULL)&&(tmp==0)){
		bRemote = false;
#ifdef DEBUG_SERIAL
		Serial.println("sgremote,0");
#endif
	}
	else if((*en==NULL)&&(tmp==1)){
		bRemote = true;
#ifdef DEBUG_SERIAL
		Serial.println("sgremote,1");
#endif
	}
}
static void rst(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	/* Set ELEVEL 2 */
	__asm("mov r0,psw\n or r0,#2\n mov psw,r0\n");
	/* Software Reset */
	__asm("brk");

}

// WIRE
static void wireb(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	Wire.begin();
	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p("wireb");
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}


static void wirebt(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	char address;
	char* en;
	int i;	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	address = (uint32_t)strtol(pparam[1],&en,0);
	if(*en!=NULL) return;

	Wire.beginTransmission(address);
	
	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p("wirebt,0x");
	Print.l((long)address,HEX);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void wireet(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	char bstop;
	char* en;
	int i;	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	bstop = (uint32_t)strtol(pparam[1],&en,0);
	if((*en!=NULL) || (bstop!=0)&&(bstop!=1)) return;

	Wire.endTransmission(bstop);
	
	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p("wireet,0x");
	Print.l((long)bstop,HEX);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}

static void wirew(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	uint8_t data;
	char* en;
	char* ptr;	

	// command sprit
	Print.init(obuf,sizeof(obuf));
	Print.p("wirew,");
	while(	(ptr = strtok(NULL,", \r\n")) != NULL)
	{
		data = (uint8_t)strtol(ptr,&en,0);
		if(*en!=NULL) return;
		Wire.write_byte(data);
		Print.p("0x");
		Print.l(data,HEX);
		Print.p(",");
	}
	Print.ln();
	
	// command process

	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}

static void wirerf(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	char address;
	char qty;	
	char bstop;
	char* en;
	int i;	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
//	Serial.println("wirerf");
	address = (uint32_t)strtol(pparam[1],&en,0);
	if(*en!=NULL) return;
//	Serial.println_long(address,HEX);
	qty = (uint32_t)strtol(pparam[2],&en,0);
	if(*en!=NULL) return;
//	Serial.println_long(qty,DEC);
	bstop = (uint32_t)strtol(pparam[3],&en,0);
	if((*en!=NULL) || (bstop!=0)&&(bstop!=1)) return;
//	Serial.println_long(bstop,DEC);

	Wire.requestFrom(address,qty,bstop);
	
	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p("wirerf,0x");
	Print.l((long)address,HEX);
	Print.p(",");
	Print.l((long)qty,DEC);
	Print.p(",");
	Print.l((long)bstop,HEX);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void wirea(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	uint8_t data;
	char* en;

	// command sprit
	Print.init(obuf,sizeof(obuf));
	Print.p("wirea,");
	Print.l((long)Wire.available(),DEC);
	Print.ln();
	// command process

	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void wirer(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int len;
	int data;
	uint8_t tmp;
	uint8_t base=10;
	char* en;
	int i;	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	tmp = (uint8_t)strtol(pparam[1],&en,0);
	if(*en==NULL) {
		if((tmp==10)||(tmp==8)||(tmp==16)) base = tmp;
	}
	
	// command sprit
	Print.init(obuf,sizeof(obuf));
	Print.p("wirer");
	len = Wire.available();
	while(len>0)
	{
		data = Wire.read();
		if(data<0) break;
		Print.p(",");
		Print.l(data,base);
		len--;
	}
	Print.ln();
	// command process
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}

// End of WIRE

// SPI
static void spib(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	SPI.begin();
	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p("spib");
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}

static void spit(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	uint8_t *ptr;
	uint8_t *en;
	int16_t data;

	Print.init(obuf,sizeof(obuf));
	Print.p("spit");
	while(	(ptr = strtok(NULL,", \r\n")) != NULL)
	{
		data = (uint8_t)strtol(ptr,&en,0);
		if(*en!=NULL) return;
		if((data>=0)&&(data<=255)) {
			data = SPI.transfer((uint8_t)data);
			Print.p(",");
			Print.l(data,DEC);
		}
	}
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void spibo(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;
	uint8_t bo;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process	
	bo = (int)strtol(pparam[1],&en,0);
	if(*en==NULL) return;
	if((bo == 0) || (bo ==1)) {
		SPI.setBitOrder(bo);
	}
	Print.init(obuf,sizeof(obuf));
	Print.p("spibo,");
	Print.l(bo,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void spidm(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;
	uint8_t dm;
	const uint8_t table[]={0,0x20,0x40,0x60};

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process	
	dm = (int)strtol(pparam[1],&en,0);
	if(*en==NULL) return;
	if((dm >= 0) && (dm < 4)) {
		SPI.setDataMode(table[dm]);
	}
	Print.init(obuf,sizeof(obuf));
	Print.p("spidm,");
	Print.l(dm,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void spicd(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;
	uint8_t cd;
	uint8_t cd_param;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process	
	cd = (int)strtol(pparam[1],&en,0);
	if(*en==NULL) return;
	switch(cd) {
	case 2:
		cd_param=0x01;
		break;
	case 4:
		cd_param=0x02;
		break;
	case 8:
		cd_param=0x04;
		break;
	case 16:
		cd_param=0x08;
		break;
	case 32:
		cd_param=0x10;
		break;
	case 64:
		cd_param=0x20;
		break;
	case 128:
		cd_param=0x40;
		break;
	default:
		return;
	}
	SPI.setClockDivider(cd_param);
	Print.init(obuf,sizeof(obuf));
	Print.p("spicd,");
	Print.l(cd,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
static void spie(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	SPI.end();
	// message output
	Print.init(obuf,sizeof(obuf));
	Print.p("spib");
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
}
void sleepCallback(void)
{
		nSleep=digitalRead(pinSleep);
}

static void bindslp(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;
	uint8_t pin;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process	
	pin = (int)strtol(pparam[1],&en,0);
	if(*en==NULL) return;
	if(pin==0)
	{
		pinSleep=0;
		drv_detachInterrupt(2);
	} else if((pin>=2)&&(pin<=19)) {
		pinSleep = pin;
		drv_attachInterrupt(digital_pin_to_port[pin], 2, sleepCallback, CHANGE,false,false);
		nSleep=digitalRead(pinSleep);
	}
	
	Print.init(obuf,sizeof(obuf));
	Print.p("bindslp,");
	Print.l(pinSleep,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
}
static void bindrcv(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	char* en;
	uint8_t pin;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);
	
	// command process	
	pin = (int)strtol(pparam[1],&en,0);
	if(*en==NULL) return;
	if(pin==0)
	{
		pinRecv=0;
	} else if((pin>=2)&&(pin<=19)) {
		pinRecv = pin;
		digitalWrite(pinRecv,0);
		pinMode(pinRecv,OUTPUT);
	}

	// command process	
	Print.init(obuf,sizeof(obuf));
	Print.p("bindrcv,");
	Print.l(pinRecv,DEC);
	Print.ln();
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}	
}

// *************************** TEST PROGRAM ********************************
void deepHALT(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	sleep(60*1000);
}

void eeprom_wpb(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	int i=0;
	char* en;
	int wpb;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process	
	wpb = (int)strtol(pparam[1],&en,0);
	if(pparam[1]==NULL) {
		goto error;
	}
	if(wpb == 0){
		pinMode(33,OUTPUT);
		digitalWrite(33,LOW);
#ifdef DEBUG_SERIAL
		Serial.println("ewp,0");
#endif
	} else if (wpb == 1) {
		pinMode(33,OUTPUT);
		digitalWrite(33,HIGH);
#ifdef DEBUG_SERIAL
		Serial.println("ewp,1");
#endif
	}
error:
	return;
}
void eeprom_write(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	int i=0;
	char* en;
	union {
		uint16_t addr16;
		uint8_t addr8[2];
	} addr;
	int data;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	if((pparam[1]==NULL)||(pparam[2]==NULL)) goto error;
	
	addr.addr16 = (int)strtol(pparam[1],&en,0);
	data = (int)strtol(pparam[2],&en,0);
	if(((addr.addr16<0)||(addr.addr16>0xFFF)) && ((data < 0) || (data > 255))) {
		goto error;
	} else {
		Wire0.beginTransmission(0x50);
		Wire0.write_byte(addr.addr8[1]);		// address upper byte
		Wire0.write_byte(addr.addr8[0]);		// address lower byte
		Wire0.write_byte((uint8_t) data);		// data write
		Wire0.endTransmission(true);
#ifdef DEBUG_SERIAL
		Serial.print("ewr,0x");
		Serial.print_long(addr.addr16,HEX);
		Serial.print(",0x");
		Serial.println_long(data,HEX);
#endif
	}
error:
	return;
}

void eeprom_read(uint8_t** pparam,SUBGHZ_MAC_PARAM* mac) {
	int i=0;
	char* en;
	union {
		uint16_t addr16;
		uint8_t addr8[2];
	} addr;
	uint16_t size,n;
	int data;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

//	Serial.println("erd");
	
	// command process
	if(pparam[1]==NULL) goto error;
	addr.addr16 = (int)strtol(pparam[1],&en,0);
	if(*en != NULL) return;
//	Serial.println_long(addr.addr16,HEX);
	size = (int)strtol(pparam[2],&en,0);
	if(*en != NULL) return;
//	Serial.println_long(size,HEX);

	if((addr.addr16<0)||(addr.addr16>0xFFF) || (size<0) || (size > 32)) {
		goto error;
	}
	Wire0.beginTransmission(0x50);
	Wire0.write_byte(addr.addr8[1]);
	Wire0.write_byte(addr.addr8[0]);
	Wire0.endTransmission(false);
	Print.init(obuf,sizeof(obuf));
	Print.p("erd,0x");
	Print.l(addr.addr16,HEX);
	Print.p(",");
	Print.l(size,DEC);
	if(size<256) {
		Wire0.requestFrom(0x50,(uint8_t)size,true);
		
		for(n=0;n<size;n++)
		{
			Print.p(",");
			data = Wire0.read();
			Print.l((long)data,HEX);
		}
		Print.ln();
	}
	if(mac==NULL)
	{
		Serial.print(obuf);
	} else {
		uint16_t dst_addr;
		dst_addr = *((uint16_t *)mac->src_addr);
		digitalWrite(TXLED,LOW);
		SubGHz.send(mac->dst_panid,dst_addr,obuf,Print.len(),NULL);
		digitalWrite(TXLED,HIGH);
	}
error:
	return;
}

void ml7396_write(uint8_t** pparam) {
	int i=0;
	char* en;
	int bank;
	int addr;
	uint8_t data;

	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	if((pparam[1]==NULL)||(pparam[2]==NULL)||(pparam[3]==NULL)) goto error;
	bank = (int)strtol(pparam[1],&en,0);
	addr = (int)strtol(pparam[2],&en,0);
	data = (int)strtol(pparam[3],&en,0);
	if(((bank<0)||(bank>3)) &&
	((addr<0)||(addr>255)) &&
	((data<0)||(data>255))) {
		goto error;
	} else {
#ifdef DEBUG_SERIAL
		Serial.print("rfw,");
		Serial.print_long(bank,DEC);
		Serial.print(",0x");
		Serial.print_long(addr,HEX);
		Serial.print(",0x");
		Serial.println_long(data,HEX);
#endif	
		phy_regwrite((uint8_t)bank, (uint8_t)addr,(uint8_t *)&data, 1);
	}
error:
	return;
}

void ml7396_read(uint8_t** pparam){
	int i=0;
	char* en;
	int bank;
	int addr;
	uint8_t data;
	
	// command sprit
	do {			
		i++;
	} while((pparam[i] = strtok(NULL,", \r\n"))!=NULL);

	// command process
	if((pparam[1]==NULL)||(pparam[2]==NULL)) goto error;
	bank = (int)strtol(pparam[1],&en,0);
	addr = (int)strtol(pparam[2],&en,0);
	if(((bank<0)||(bank>3)) &&
	((addr<0)||(addr>255))) {
		goto error;
	} else {		
		phy_regread( bank,  addr,  &data,  1);
#ifdef DEBUG_SERIAL
		Serial.print("rfr,");
		Serial.print_long(bank,DEC);
		Serial.print(",0x");
		Serial.print_long(addr,HEX);
		Serial.print(",");
#endif	
		Serial.print("0x");
		Serial.println_long((long)data,HEX);
	}
error:
	return;
}
// *************************** TEST PROGRAM ********************************

void send_millis(uint8_t** pparam){
	Serial.print("millis,");
	Serial.println_long(millis(),DEC);
}

void command_decoder(uint8_t* pcmd,uint8_t** pparam,SUBGHZ_MAC_PARAM* mac)
{
	int i=0;
	pparam[i] = strtok(pcmd,", \r\n");
//do {			
//			i++;
//  		} while((param[i] = strtok(NULL,", \r\n"))!=NULL);
         if(strncmp(pparam[0],CMD_PIN_MODE,16)==0) gpio_set(pparam,mac);
	else if(strncmp(pparam[0],CMD_DIGITAL_READ,16)== 0) gpio_read(pparam,mac);
	else if(strncmp(pparam[0],CMD_DIGITAL_WRITE,16)== 0) gpio_write(pparam,mac);
	else if(strncmp(pparam[0],CMD_SUBGHZ_INIT,16)== 0) sgi(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_BEGIN,16)== 0) sgb(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_SEND,16)== 0) sgs(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_CONTINUE_SENDING,16)== 0) sgcs(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_SEND_HOPPING,16)== 0) sghp(pparam);
	else if(strncmp(pparam[0],CMD_WRITE_DATA,16)== 0) write_data(pparam,mac);
	else if(strncmp(pparam[0],CMD_WRITE_BINARY,16)== 0) write_binary(pparam,mac);
	else if(strncmp(pparam[0],CMD_SUBGHZ_RXENABLE,16)== 0) sgre(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_RX_AUTO,16)== 0) sgra(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_RXDISABLE,16)== 0) sgrd(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_SET_SEND_MODE,16)== 0) sgssm(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_GET_SEND_MODE,16)== 0) sggsm(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_CLOSE,16)== 0) sgc(pparam);  		
	else if(strncmp(pparam[0],CMD_SUBGHZ_GET_MY_ADDRESS,16)== 0) sggma(pparam,mac);
	else if(strncmp(pparam[0],CMD_SUBGHZ_GET_MY_ADDR64,16)== 0) sggma64(pparam,mac);
	else if(strncmp(pparam[0],CMD_SUBGHZ_SET_MY_ADDRESS,16)== 0) sgsma(pparam,mac);
	else if(strncmp(pparam[0],CMD_SUBGHZ_SET_ANTSW,16)== 0) sgansw(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_GET_STATUS,16)== 0) sggs(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_READ,16)== 0) sgr(pparam);
	else if(strncmp(pparam[0],CMD_SUBGHZ_READ_BINARY,16)== 0) sgrb(pparam);
	else if(strncmp(pparam[0],CMD_SET_BAUD,16)== 0) sb(pparam,mac);
	else if(strncmp(pparam[0],CMD_SUBGHZ_REMOTE,16)== 0) sgremote(pparam,mac);
	else if(strncmp(pparam[0],CMD_RESET,16)== 0) rst(pparam,mac);
	else if(strncmp(pparam[0],CMD_WIRE_BEGIN,16)== 0) wireb(pparam,mac);
	else if(strncmp(pparam[0],CMD_WIRE_BEGIN_TRANSMITTION,16)== 0) wirebt(pparam,mac);
	else if(strncmp(pparam[0],CMD_WIRE_END_TRANSMITTION,16)== 0) wireet(pparam,mac);
	else if(strncmp(pparam[0],CMD_WIRE_WRITE,16)== 0) wirew(pparam,mac);
	else if(strncmp(pparam[0],CMD_WIRE_AVAILABLE,16)== 0) wirea(pparam,mac);
	else if(strncmp(pparam[0],CMD_WIRE_REQUEST_FROM,16)== 0) wirerf(pparam,mac);
	      if(strncmp(pparam[0],CMD_WIRE_READ,16)== 0) wirer(pparam,mac);
	else if(strncmp(pparam[0],CMD_SPI_BEGIN,16)== 0) spib(pparam,mac);
	else if(strncmp(pparam[0],CMD_SPI_TRANSFER,16)== 0) spit(pparam,mac);
	else if(strncmp(pparam[0],CMD_SPI_BIT_ORDER,16)== 0) spibo(pparam,mac);
	else if(strncmp(pparam[0],CMD_SPI_DATA_MODE,16)== 0) spidm(pparam,mac);
	else if(strncmp(pparam[0],CMD_SPI_CLOCK_DIVIDER,16)== 0) spicd(pparam,mac);
	else if(strncmp(pparam[0],CMD_SPI_END,16)== 0) spie(pparam,mac);
	else if(strncmp(pparam[0],CMD_BIND_SLEEP_PIN,16)== 0) bindslp(pparam,mac);
	else if(strncmp(pparam[0],CMD_BIND_RECV_PIN,16)== 0) bindrcv(pparam,mac);
	else if(strncmp(pparam[0],CMD_FLASH_WR,16)== 0) fwr(pparam,mac);
	else if(strncmp(pparam[0],CMD_FLASH_RD,16)== 0) frd(pparam,mac);
	else if(strncmp(pparam[0],CMD_FLASH_ERASE,16)== 0) fer(pparam,mac);
// *************************** TEST PROGRAM ********************************
	else if(strncmp(pparam[0],CMD_DEEP_HALT,16)==0) deepHALT(pparam,mac);
	else if(strncmp(pparam[0],CMD_EEPROM_WPB,16)==0) eeprom_wpb(pparam,mac);
	else if(strncmp(pparam[0],CMD_EEPROM_WRITE,16)==0) eeprom_write(pparam,mac);
	else if(strncmp(pparam[0],CMD_EEPROM_READ,16)==0) eeprom_read(pparam,mac);
	else if(strncmp(pparam[0],CMD_RF_WRITE,16)==0) ml7396_write(pparam);
	else if(strncmp(pparam[0],CMD_RF_READ,16)==0) ml7396_read(pparam);
// *************************** TEST PROGRAM ********************************
	else if(strncmp(pparam[0],CMD_MILLIS,16)==0) send_millis(pparam);
}
static char* param[16];

void setup() {
  // put your setup code here, to run once:
  unsigned long baud;
//  SubGHz.init();
  baud = Flash.read(0,0);
  baud = (baud << 16) + Flash.read(0,1);
  Serial.begin(115200);
  Serial.println_long(baud,DEC);
  delay(100);
  Serial.end();

  if(!check_baud(baud))
  {
    baud = 115200;
 	Flash.erase(0);
 	Flash.write(0,0,(unsigned short)((baud>>16)&0x0000FFFF));
	Flash.write(0,1,(unsigned short)(baud&0x0000FFFF));
	delay(100);
  }
  Serial.begin(baud);
  Serial.println("Welcome"); 
  pinMode(TXLED,OUTPUT);
  pinMode(RXLED,OUTPUT);
  SubGHz.init();
}

void loop() {
	static int bufp=0;

	if(!nSleep) wait_event(&nSleep);
  // put your main code here, to run repeatedly:
  if(Serial.peek() >= 0) {
  	cmdbuf[bufp] = (char) Serial.read();
	if((cmdbuf[bufp] == '\r')||(cmdbuf[bufp] == '\n')) {
		int i=0;
		cmdbuf[bufp+1] = NULL;
  		bufp=0;
		command_decoder(cmdbuf,param,NULL);
  	} else {
	  	bufp++;
	}
  }
  if(sgRxEvent==true)
  {
  	int result;
	result = SubGHz.readData(rbuf,sizeof(rbuf));
	if(result > 0)
	{
		SUBGHZ_MAC_PARAM mac;
		SUBGHZ_STATUS rx;
		SubGHz.getStatus(NULL,&rx);
		SubGHz.decMac(&mac,rbuf,result);
		mac.payload[mac.payload_len]=NULL;
		Serial.println(mac.payload);
	// command process
		command_decoder(mac.payload,param,&mac);
	}
	sgRxEvent=false;
	if(pinRecv) digitalWrite(pinRecv,LOW);
  	digitalWrite(RXLED,HIGH);
  }
	if((sgRxAuto)&&(digitalRead(RXLED)==LOW))
	{
		sgout();
	}
}
