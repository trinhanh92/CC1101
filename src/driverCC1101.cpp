#include "driverCC1101.h"
#include "stdio.h"
using namespace CC_ns;

uint8_t paTable[8] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	//0dBm

SPI_ns::SPI_params_t CC_SPI_params_struct;
SPI_ns::SM_device_t deviceType = SPI_ns::cc1101_1;
GPIO_InitTypeDef GPIO_GDO;

//txData wrData;
//rxData rdData;

/*
 *
 */
driverCC1101::driverCC1101() {
	SPI_ptr = NULL;
}

/*
 *	@Initialize CC1101.
 */
void driverCC1101::initCC1101(CC_SPI_params_s* CC_conf) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_StructInit(&GPIO_GDO);

	GPIO_GDO.GPIO_Pin = GDO0;
	GPIO_GDO.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_GDO.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GDO0_Port, &GPIO_GDO);

	GPIO_GDO.GPIO_Pin = GDO2;
	GPIO_GDO.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_GDO.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GDO2_Port, &GPIO_GDO);

	SPI_ptr = CC_conf->CC_SPI_ptr;
	deviceType = CC_conf->deviceType;

	reinitSPI();
	SPI_ptr->SM_device_attach(deviceType);

	powerUpOnResetCC1101();		//reset cc1101
	halRfWriteSettings();		//setting registers
	halWriteBurstReg(CC110x_PATABLE, paTable, 8);	//setting patable.

	return;
}

/*
 *	@
 */
void driverCC1101::powerUpOnResetCC1101() {
	SPI_ptr->SM_device_deselect(deviceType);
	SPI_ptr->SM_device_select(deviceType);
	SPI_ptr->SM_device_deselect(deviceType);

	halWait(51);

	resetCC1101();

	return;
}

/*
 *	@Reset CC101.
 */
void driverCC1101::resetCC1101() {
	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, CC110x_SRES);

	SPI_ptr->SM_device_deselect(deviceType);

	return;
}

/*
 *	@Re-initialize to use SPI for CC1101.
 */
void driverCC1101::reinitSPI() {
	CC_SPI_params_struct.baudRatePrescaler = SPI_BaudRatePrescaler_4;
	CC_SPI_params_struct.CPOL = SPI_CPOL_Low;
	CC_SPI_params_struct.CPHA = SPI_CPHA_1Edge;
	CC_SPI_params_struct.dataSize = SPI_DataSize_8b;
	CC_SPI_params_struct.firstBit = SPI_FirstBit_MSB;
	CC_SPI_params_struct.mode = SPI_Mode_Master;
	CC_SPI_params_struct.direction = SPI_Direction_2Lines_FullDuplex;
	CC_SPI_params_struct.nss = SPI_NSS_Soft;
	CC_SPI_params_struct.crcPoly = 7;

	SPI_ptr->init(&CC_SPI_params_struct);
}

/*
 *	@Send strobe commands to CC1101.
 */
void driverCC1101::halSendStrobe(uint8_t cmdStrobe) {
	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, cmdStrobe);

	SPI_ptr->SM_device_deselect(deviceType);

	return;
}

/*
 *	@Write one byte-value to addr of CC1101.
 */
void driverCC1101::halWriteSingleReg(uint8_t addr, uint8_t value) {
	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, addr);
	SPI_ptr->M2F_sendAndGet_blocking(deviceType, value);

	SPI_ptr->SM_device_deselect(deviceType);

	return;
}

/*
 *	@Write a buffer-value (burst) to addr of CC1101.
 */
void driverCC1101::halWriteBurstReg(uint8_t addr, uint8_t* txBuf, uint8_t length) {
	uint8_t i;

	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, addr|WRITE_BURST);
	for (i = 0; i < length; i++) {
		SPI_ptr->M2F_sendAndGet_blocking(deviceType, txBuf[i]);
	}

	SPI_ptr->SM_device_deselect(deviceType);

	return;
}

/*
 *	@Read one byte-value from addr of CC1101.
 */
uint8_t driverCC1101::halReadSingleReg(uint8_t addr) {
	uint8_t retval;

	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, addr|READ_SINGLE);
	retval = SPI_ptr->M2F_sendAndGet_blocking(deviceType, 0xFF);

	SPI_ptr->SM_device_deselect(deviceType);

	return retval;
}

/*
 *	@Read into a buffer-value from addr of CC1101.
 */
uint8_t driverCC1101::halReadBurstReg(uint8_t addr, uint8_t *rxBuf, uint8_t length) {
	uint8_t i;

	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, addr|READ_BURST);
	for (i = 0; i < length; i++) {
		rxBuf[i] = SPI_ptr->M2F_sendAndGet_blocking(deviceType, 0xFF);
	}

	SPI_ptr->SM_device_deselect(deviceType);

	return *rxBuf;
}

/*
 *	@Read a status register at addr of CC1101.
 */
uint8_t driverCC1101::halReadSatusReg(uint8_t addr) {
	uint8_t retval;

	SPI_ptr->SM_device_select(deviceType);
	waitMisoLow();

	SPI_ptr->M2F_sendAndGet_blocking(deviceType, addr|READ_BURST);
	retval = SPI_ptr->M2F_sendAndGet_blocking(deviceType, 0xFF);

	SPI_ptr->SM_device_deselect(deviceType);

	return retval;
}

/*
 *	@Setting value for CC1101 registers
 */
void driverCC1101::halRfWriteSettings() {
	// Write register settings
	halWriteSingleReg(CC110x_FSCTRL1, 	rfSettings.FSCTRL1);
	halWriteSingleReg(CC110x_FSCTRL0, 	rfSettings.FSCTRL0);
	halWriteSingleReg(CC110x_FREQ2, 	rfSettings.FREQ2);
	halWriteSingleReg(CC110x_FREQ1, 	rfSettings.FREQ1);
	halWriteSingleReg(CC110x_FREQ0, 	rfSettings.FREQ0);
	halWriteSingleReg(CC110x_MDMCFG4, 	rfSettings.MDMCFG4);
	halWriteSingleReg(CC110x_MDMCFG3, 	rfSettings.MDMCFG3);
	halWriteSingleReg(CC110x_MDMCFG2, 	rfSettings.MDMCFG2);
	halWriteSingleReg(CC110x_MDMCFG1, 	rfSettings.MDMCFG1);
	halWriteSingleReg(CC110x_MDMCFG0, 	rfSettings.MDMCFG0);
	halWriteSingleReg(CC110x_CHANNR, 	rfSettings.CHANNR);
	halWriteSingleReg(CC110x_DEVIATN, 	rfSettings.DEVIATN);
	halWriteSingleReg(CC110x_FREND1, 	rfSettings.FREND1);
	halWriteSingleReg(CC110x_FREND0, 	rfSettings.FREND0);
	halWriteSingleReg(CC110x_MCSM0 , 	rfSettings.MCSM0 );
	halWriteSingleReg(CC110x_FOCCFG, 	rfSettings.FOCCFG);
	halWriteSingleReg(CC110x_BSCFG, 	rfSettings.BSCFG);
	halWriteSingleReg(CC110x_AGCCTRL2, 	rfSettings.AGCCTRL2);
	halWriteSingleReg(CC110x_AGCCTRL1, 	rfSettings.AGCCTRL1);
	halWriteSingleReg(CC110x_AGCCTRL0, 	rfSettings.AGCCTRL0);
	halWriteSingleReg(CC110x_FSCAL3, 	rfSettings.FSCAL3);
	halWriteSingleReg(CC110x_FSCAL2, 	rfSettings.FSCAL2);
	halWriteSingleReg(CC110x_FSCAL1, 	rfSettings.FSCAL1);
	halWriteSingleReg(CC110x_FSCAL0, 	rfSettings.FSCAL0);
	halWriteSingleReg(CC110x_FSTEST, 	rfSettings.FSTEST);
	halWriteSingleReg(CC110x_TEST2, 	rfSettings.TEST2);
	halWriteSingleReg(CC110x_TEST1, 	rfSettings.TEST1);
	halWriteSingleReg(CC110x_TEST0, 	rfSettings.TEST0);
	halWriteSingleReg(CC110x_FIFOTHR, 	rfSettings.FIFOTHR);
	halWriteSingleReg(CC110x_IOCFG2, 	rfSettings.IOCFG2);
	halWriteSingleReg(CC110x_IOCFG0, 	rfSettings.IOCFG0);
	halWriteSingleReg(CC110x_PKTCTRL1, 	rfSettings.PKTCTRL1);
	halWriteSingleReg(CC110x_PKTCTRL0, 	rfSettings.PKTCTRL0);
	halWriteSingleReg(CC110x_ADDR, 		rfSettings.ADDR);
	halWriteSingleReg(CC110x_PKTLEN, 	rfSettings.PKTLEN);

	return;
}

/*
 *
 */
void driverCC1101::halWait(uint16_t timeout) {
	while(timeout--);

	return;
}

/*
 *
 */
void driverCC1101::mDelay(uint16_t time) {
	int i,j;
	for(i=0;i<time;i++)
		for(j=0;j<1275;j++);
}

/*
 *
 */
void driverCC1101::waitMisoLow() {
	while(SPI_ptr->MisoLevelGet() == SET);

	return;
}

/*
 *
 */
//void driverCC1101::halSendRfPacket(uint8_t *txBuf, uint8_t length) {
//	halWriteBurstReg(CC110x_TXFIFO, txBuf, length);
//
//	halSendStrobe(CC110x_STX);
//
//	//wait sync word is sent
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == RESET);
//
// 	//wait end of packet
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == SET);
//
//	halSendStrobe(CC110x_SIDLE);
//	halSendStrobe(CC110x_SFTX);
//
//	return;
//}
//
//bool driverCC1101::halReceiveRfPacket(uint8_t *rxBuf, uint8_t length) {
//	uint8_t status[2];
//	uint8_t packetLength;
//
//	halSendStrobe(CC110x_SRX);
//
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == RESET);
//
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == SET);
//
//	if ((halReadSatusReg(CC110x_RXBYTES) & BYTES_IN_RXFIFO)) {
//		// Read length byte
//		packetLength = halReadSingleReg(CC110x_RXFIFO);
//
//		// Read data from RX FIFO and store in rxBuffer
//		if (packetLength <= length) {
//			// Write the packet length to the first position of rxBuf[]
//			rxBuf[0] = packetLength;
//
//			// Write all data in RX_FIFO to rxBuf[1:packetLength]
//			halReadBurstReg(CC110x_RXFIFO, rxBuf + 1, packetLength);
//
//			// Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
//			halReadBurstReg(CC110x_RXFIFO, status, 2);
//
//			// Make sure that the radio is in IDLE state before flushing the FIFO
//			// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
//			halSendStrobe(CC110x_SIDLE);
//
//			// Flush RX FIFO
//			halSendStrobe(CC110x_SFRX);
//
//			// MSB of LQI is the CRC_OK bit
//			return (status[1] & CRC_OK);
//		} else {
//
//			// Make sure that the radio is in IDLE state before flushing the FIFO
//			// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
//			halSendStrobe(CC110x_SIDLE);
//
//			// Flush RX FIFO
//			// <If RX autoflush mode is set, RX_FIFO get automatically flushed>
//			halSendStrobe(CC110x_SFRX);
//
//			return false;
//		}
//	} else {
//		return false;
//	}
//}
//
///*
// *
// */
//void driverCC1101::halSendVariablePacket(uint8_t *txBuf, uint8_t length) {
//	uint8_t loop = 0;
//
//	halSendStrobe(CC110x_SIDLE);
//	halSendStrobe(CC110x_SFTX);
//
//	/*
//	 * @ Init data stuctrure.
//	 */
//	wrData.bufferPtr = txBuf;
//	wrData.bytesLeft = length + 1;
//	wrData.iterations = 0;
//	wrData.writeRemainingDataFlag = false;
//
//	/*
//	 * @ Write 64bytes to the TX_FIFO and transmit over rf.
//	 */
//	halWriteBurstReg(CC110x_TXFIFO, txBuf, FIFO_SIZE);
//	halSendStrobe(CC110x_STX);
//
//	/*
//	 * @ Remaining byte need to be transmited.
//	 */
//	wrData.bytesLeft = length + 1 - FIFO_SIZE;
//
//	/*
//	 * @ Update pointer of txBuf.
//	 */
//	wrData.bufferPtr += FIFO_SIZE;
//
//	/*
//	 * @ Remaining iterations.
//	 */
//	wrData.iterations = wrData.bytesLeft/FIFO_THRSH;
//	loop = wrData.iterations + 1;
//
//	/*
//	 * @ Check remaining data flag to finish threshold intertupt.
//	 */
//	if(wrData.iterations == 0)
//		wrData.writeRemainingDataFlag = true;
//	else wrData.writeRemainingDataFlag = false;
//
//	/*
//	 * @ wait sync word is sent.
//	 */
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == RESET);
//
// 	/*
// 	 * @ wait end of packet.
// 	 */
// 	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == SET) {
//
// 		/*
// 		 * @ var loop is used to end the whileloop when send remaining bytes.
// 		 */
// 		while (GPIO_ReadInputDataBit(GPIOB, GDO2) == SET && loop != 0);
// 		loop--;
// 		if(wrData.bytesLeft != 0) {
//			if(wrData.writeRemainingDataFlag == true) {
//				/*
//				 * @ Write remaining data to TX_FIFO and transmit over rf.
//				 */
//				halWriteBurstReg(CC110x_TXFIFO, wrData.bufferPtr, wrData.bytesLeft);
//				wrData.bytesLeft = 0;
//			}else {
//				/*
//				 * @ Write 60bytes data to TX_FIFO and transmit over rf.
//				 */
//				halWriteBurstReg(CC110x_TXFIFO, wrData.bufferPtr, FIFO_THRSH);
//
//				/*
//				 * @ update wrData.
//				 */
//				wrData.bytesLeft -= FIFO_THRSH;
//				wrData.bufferPtr += FIFO_THRSH;
//				wrData.iterations -= 1;
//				if(wrData.iterations == 0)
//					wrData.writeRemainingDataFlag = true;
//			}
// 		}
// 	}
//
//	return;
//}
//
///*
// *
// */
//bool driverCC1101::halReceiveVariablePacket(uint8_t *rxBuf) {
//	uint8_t status[2];
//	uint8_t packetLength;
//
//	/*
//	 * @ init data structure.
//	 */
//	rdData.bytesLeft = 0;
//	rdData.bufferPtr = rxBuf;
//
//	/*
//	 * @ Receiving state.
//	 */
//	halSendStrobe(CC110x_SRX);
//
//	/*
//	 * @ wait sync word is sent.
//	 */
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == RESET);
//
//	/*
//	 * @ wait for receiving 2bytes before read the lengthbyte.
//	 */
//	while((halReadSatusReg(CC110x_RXBYTES)&BYTES_IN_RXFIFO) < 2);
//
//	/*
//	 * @ Read the length byte in RX_FIFO and store it into the rxBuf[0].
//	 */
//	packetLength = halReadSingleReg(CC110x_RXFIFO);
//	rxBuf[0] = packetLength;
//
//	/*
//	 * @ Number of bytes left to read.
//	 */
//	rdData.bytesLeft = packetLength;
//
//	/*
//	 * @ Update pointer of rxBuf.
//	 */
//	rdData.bufferPtr++;
//
//	/*
//	 * @ wait end of packet.
//	 */
//	while (GPIO_ReadInputDataBit(GPIOB, GDO0) == SET) {
//		while (GPIO_ReadInputDataBit(GPIOB, GDO2) == RESET) {
//			if(GPIO_ReadInputDataBit(GPIOB, GDO0) == RESET)
//				break;
//		}
//
//		if(GPIO_ReadInputDataBit(GPIOB, GDO0) == RESET) {
//			/*
//			 * @ Read remaining bytes in RX_FIFO.
//			 */
//			if((halReadSatusReg(CC110x_RXBYTES)&BYTES_IN_RXFIFO) != 0)
//				halReadBurstReg(CC110x_RXFIFO, rdData.bufferPtr, rdData.bytesLeft);
//
//			rdData.bytesLeft = 0;
//
//			/*
//			 * @ Read status bytes (status[0] = RSSI, status[1] = LQI).
//			 */
//			halReadBurstReg(CC110x_RXFIFO, status, 2);
//
//			break;
//		}else {
//			/*
//			 * @ Write (FIFO_THRSH-1) bytes data in RX_FIFO to rxBuf[1:FIFO_THRSH-1].
//			 */
//			halReadBurstReg(CC110x_RXFIFO, rdData.bufferPtr, (FIFO_THRSH - 1));
//
//			/*
//			 * @ Update pointer of rxBuf[] and num of bytes left to read
//			 */
//			rdData.bufferPtr += FIFO_THRSH - 1;
//			rdData.bytesLeft -= FIFO_THRSH - 1;
//		}
//	}
//
//	return (status[1]&CRC_OK);
//}
