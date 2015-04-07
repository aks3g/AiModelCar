/*
 * i2c_driver.cpp
 *
 * Created: 2013/09/15 14:57:10
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/gpio_driver.h>
#include <xmegaE5/i2c_driver.h>

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static volatile I2C_MODE sCurrentMode = I2C_NOT_INITIALIZED;

static volatile uint8_t sI2cSlaveMode;
static volatile uint8_t sIsFirstByte;
static volatile I2C_INIT_OPT sSlaveContext;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t _sInitialize_i2cSlave(I2C_INIT_OPT *opt);
static uint8_t _sInitialize_i2cMaster(uint32_t clk, uint32_t sysclkHz);
static uint8_t _sI2c_txSlaveAddress(uint8_t slaveAddress);
static uint8_t _sI2c_txByte(uint8_t data);
static uint8_t _sI2c_rxByte(uint8_t *data, uint8_t ack);

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#define I2C_IS_NACK()				(TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm)
#define I2C_WAIT_FOR_TX_DONE()		{ while (!(TWIC.MASTER.STATUS & TWI_MASTER_WIF_bm)); }
#define I2C_WAIT_FOR_RX_DONE()		{ while (!(TWIC.MASTER.STATUS & TWI_MASTER_RIF_bm)); }
#define I2C_STOP_CONDITION()		(TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc)
#define I2C_RESTART()				(TWIC.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc)

#define ENABLE_I2C_MASTER_INTERRUPT()	{TWIC.MASTER.CTRLA |=  (TWI_MASTER_INTLVL_MED_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm);}
#define DISABLE_I2C_MASTER_INTERRUPT()	{TWIC.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_MED_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm);}

/*---------------------------------------------------------------------------*/
#define I2C_SLAVE_CMD_COMPLETE(ack)	(TWIC.SLAVE.CTRLB = (TWI_SLAVE_CMD_COMPTRANS_gc | ((ack) ? TWI_MASTER_ACKACT_bm : 0)))
#define I2C_SLAVE_CMD_RESPONSE(ack)	(TWIC.SLAVE.CTRLB = (TWI_SLAVE_CMD_RESPONSE_gc  | ((ack) ? TWI_MASTER_ACKACT_bm : 0)))


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_i2c(uint32_t clk, uint32_t sysclkHz, I2C_MODE mode, I2C_INIT_OPT *opt)
{
	uint8_t ret = I2C_OK;
	
	//J �����킩��񃂁[�h�w�肵�Ă����玀��
	if ((mode != I2C_SLAVE) && (mode != I2C_MASTER) ) {
		return I2C_INVALID_MODE;	
	} 

	//J �X���[�u�f�o�C�X�Ƃ��ď�����
	if (mode == I2C_SLAVE) {
		ret = _sInitialize_i2cSlave(opt);
	} else if (mode == I2C_MASTER) {
		ret = _sInitialize_i2cMaster(clk, sysclkHz);		
	}

	if (ret == I2C_OK) {
		sCurrentMode= mode;
	}
	
	return ret;
}

#define SUPPORT_I2C_MASTER_INTERRUPT

#ifdef SUPPORT_I2C_MASTER_INTERRUPT
/*---------------------------------------------------------------------------*/
static uint8_t  sI2cMutex = 0;

static uint8_t sSlaveAddress = 0;
static uint8_t  *sTxBuf = NULL;
static uint32_t sTxLen  = 0;
static uint8_t  *sRxBuf = NULL;
static uint32_t sRxLen  = 0;
static uint32_t sTxCounter = 0;
static uint32_t sRxCounter = 0;
static i2c_done_callback sI2cDoneCb = NULL;

uint8_t i2c_txRxBytes(uint8_t slaveAddress, uint8_t *txBuf, uint32_t txLen, uint8_t *rxBuf, uint32_t rxLen, i2c_done_callback cb)
{
	if (sCurrentMode != I2C_MASTER) {
		return I2C_MODULE_IS_NOT_MASTER_MODE;
	}

	if ( ((txBuf==NULL) && (txLen!=0)) || ((rxBuf==NULL) && (rxLen!=0)) ) {
		return I2C_BUFFER_IS_NULL_PTR;
	}

	//J I2C�f�o�C�X�̌�ʐ���
	bool gotMutex = false;
	Disable_Int();
	if (sI2cMutex == 0) {
		sI2cMutex = 1; // In use
		gotMutex = true;
	} else {
		
	}
	Enable_Int();
		
	if (!gotMutex) {
		return I2C_IS_IN_USED;	
	}

	//J �R���e�L�X�g�����e
	sSlaveAddress = slaveAddress;

	sTxCounter = 0;
	sRxCounter = 0;

	sTxBuf = txBuf;
	sRxBuf = rxBuf;

	sTxLen = txLen;
	sRxLen = rxLen;
	
	sI2cDoneCb = cb;
	
	//J �����݂�L��������
	ENABLE_I2C_MASTER_INTERRUPT();
	
	//J Slave Address ��@�����ŃX�^�[�g����	
	int8_t ret = I2C_OK;
	if (sTxLen > 0) {
		TWIC.MASTER.ADDR = ((slaveAddress << 1) | 0);
	} else {
		TWIC.MASTER.ADDR = ((slaveAddress << 1) | 1);
	}
	
	
	return ret;
}
#endif/* SUPPORT_I2C_MASTER_INTERRUPT */

/*---------------------------------------------------------------------------*/
uint8_t i2c_txRxBytes(uint8_t slaveAddress, uint8_t *txBuf, uint32_t txLen, uint8_t *rxBuf, uint32_t rxLen)
{
	uint8_t ret = I2C_OK;
	
	if (sCurrentMode != I2C_MASTER) {
		return I2C_MODULE_IS_NOT_MASTER_MODE;
	}

	if ( ((txBuf==NULL) && (txLen!=0)) || ((rxBuf==NULL) && (rxLen!=0)) ) {
		return I2C_BUFFER_IS_NULL_PTR;
	}

	//J I2C�f�o�C�X�̌�ʐ���
	bool gotMutex = false;
	Disable_Int();
	if (sI2cMutex == 0) {
		sI2cMutex = 1; // In use
		gotMutex = true;
		} else {
		
	}
	Enable_Int();
	
	if (!gotMutex) {
		return I2C_IS_IN_USED;
	}

	/*J Write�t�F�[�Y*/
	if (txLen != 0) {
		ret = _sI2c_txSlaveAddress((slaveAddress << 1) | 0);
		if (ret != I2C_OK) {
			goto END;
		}

		for (uint32_t i=0 ; i<txLen ; ++i) {
			_sI2c_txByte(*txBuf++);
		}	
	}

	if (rxLen != 0) {
		if (txLen != 0) {
			//J Restart �ǂ�����̂��������̂��B
			I2C_RESTART();
			ret = _sI2c_txSlaveAddress((slaveAddress << 1) | 1);
			if (ret != I2C_OK) {
				goto END;
			}
		} else {
			//J Start
			ret = _sI2c_txSlaveAddress((slaveAddress << 1) | 1);
			if (ret != I2C_OK) {
				goto END;
			}
		}			

		for (uint32_t i=0 ; i<rxLen ; ++i) {
			_sI2c_rxByte(rxBuf++, ((i+1)==rxLen ? 0:1));
		}

	} else {
		I2C_STOP_CONDITION();	
	}

	Disable_Int();
	sI2cMutex = 0; // Release
	Enable_Int();

	return I2C_OK;
END:
	I2C_STOP_CONDITION();
	
	Disable_Int();
	sI2cMutex = 0; // Release
	Enable_Int();

	return ret;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t _sInitialize_i2cSlave(I2C_INIT_OPT *opt)
{
	if (opt == NULL || opt->startCB == NULL || opt->stopCB == NULL || opt->masterRxCB == NULL || opt->masterTxCB == NULL || opt->ownAddress == 0x00) {
		return I2C_INVALUID_ARGS;
	}
	
	//J �R�[���o�b�N�ނ�o�^
	sSlaveContext.startCB    = opt->startCB;
	sSlaveContext.stopCB     = opt->stopCB;
	sSlaveContext.masterRxCB = opt->masterRxCB;
	sSlaveContext.masterTxCB = opt->masterTxCB;


	/*J 18.10.1 CTRL ? Control register */
	TWIC.CTRL = 0x00;
	
	/* 18.12.1 CTRLA ? Control register A */
	TWIC.SLAVE.CTRLA = TWI_SLAVE_INTLVL_MED_gc | TWI_SLAVE_DIEN_bm | TWI_SLAVE_APIEN_bm | TWI_SLAVE_ENABLE_bm | TWI_SLAVE_PIEN_bm;

	/* 18.12.2 CTRLB ? Control register B */
	//J �^�C���A�E�g�͎g��Ȃ�
	TWIC.SLAVE.CTRLB = 0x00;
	
	/* 18.12.4 ADDR ? Address register */
	TWIC.SLAVE.ADDR = (opt->ownAddress << 1) & 0xFE;

	/* 18.12.3 STATUS ? Status register */
	TWIC.SLAVE.STATUS;

	return I2C_OK;
}


/*---------------------------------------------------------------------------*/
static uint8_t _sInitialize_i2cMaster(uint32_t clk, uint32_t sysclkHz)
{
	//J �w�肳�ꂽ�N���b�N�����邩�`�F�b�N
	//J 18.11.5 BAUD ? Baud Rate register �����Ɍv�Z��������
	uint32_t baud = (sysclkHz /(2 * clk)) - 5;
	if (baud > 0xff) {
		return I2C_INVALID_CLOCK;
	}

	/*J 18.10.1 CTRL ? Control register */
	TWIC.CTRL = 0x00;
	
	/*J 18.11 Register description ? TWI master */
	TWIC.MASTER.CTRLA = TWI_MASTER_ENABLE_bm; //J �Ƃ肠�����`�L���Ȃ̂Ŋ����݂͎g��Ȃ�

	/*J 18.11.2 CTRLB ? Control register B */
	TWIC.MASTER.CTRLB = 0x00; //J �t���[����I�Ȏ��͂��Ȃ�
	
	/*J 18.11.3 CTRLC ? Control register C */
	TWIC.MASTER.CTRLC; //J �����ł͉������Ȃ�

	/*J 18.11.5 BAUD ? Baud Rate registers */
	TWIC.MASTER.BAUD = baud;

	/*J 18.11.4 STATUS ? Status register */
	TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc; //J �o�X���A�C�h����ԂɕύX


	sI2cMutex = 0;

	return I2C_OK;
}


/*---------------------------------------------------------------------------*/
static uint8_t _sI2c_txSlaveAddress(uint8_t slaveAddress)
{
	// send address
	TWIC.MASTER.ADDR = slaveAddress;

	// wait for sending/Receiving
	if (slaveAddress & 0x01) {
		I2C_WAIT_FOR_RX_DONE();
	} else {
		I2C_WAIT_FOR_TX_DONE();
	}

	return (I2C_IS_NACK() ? I2C_ERROR_NACK : I2C_OK);
}

/*---------------------------------------------------------------------------*/
static uint8_t _sI2c_txByte(uint8_t data)
{
	// send address
	TWIC.MASTER.DATA = data;

	// wait for sending
	I2C_WAIT_FOR_TX_DONE(); //?

	return (I2C_IS_NACK() ? I2C_ERROR_NACK : I2C_OK);
}

/*---------------------------------------------------------------------------*/
static uint8_t _sI2c_rxByte(uint8_t *data, uint8_t ack)
{
	I2C_WAIT_FOR_RX_DONE();

	*data = TWIC.MASTER.DATA;

	if (ack) {
		TWIC.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
	} else {
		TWIC.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
	}

	return I2C_OK;
}



/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
//J I2C Slave�̊����ݏ���
ISR(TWIC_TWIS_vect)
{
	//J �����݃X�e�[�^�X���擾
	uint8_t status = TWIC.SLAVE.STATUS;

	/* Bit 6 ? APIF: Address/Stop Interrupt Flag */
	//J Slave Address���ݒ肵�����m�ƈ�v�����痧��
	//J CtrlA��PIEN�������Ă���΁ASTOP Condition�ł�����
	if (TWIC.SLAVE.STATUS & TWI_SLAVE_APIF_bm) {
		/* Bit 0 ? AP: Slave Address or Stop */
		//J 1: Slave Address was Hit
		if (TWIC.SLAVE.STATUS & TWI_SLAVE_AP_bm) {
			/* Bit 1 ? DIR: Read/Write Direction */
			// Master Read Operation
			if (TWIC.SLAVE.STATUS & TWI_SLAVE_DIR_bm) {
				sIsFirstByte = 1;
				sI2cSlaveMode = I2C_MASTER_READ;
			}
			// Master Write Operation
			else {
				sI2cSlaveMode = I2C_MASTER_WRITE;
			}

			//J Callback to Applicatioin
			if (sSlaveContext.startCB != NULL) {
				(void) sSlaveContext.startCB(sI2cSlaveMode);
			}

			I2C_SLAVE_CMD_RESPONSE(I2C_SLAVE_RESPONCE_ACK);
		}
		//J 0: Stop Condition
		else {
			//J Callback to Applicatioin
			if (sSlaveContext.stopCB != NULL) {
				sSlaveContext.stopCB();
			}

			// Clear Flag
			TWIC.SLAVE.STATUS = TWI_SLAVE_APIF_bm;

			sI2cSlaveMode = I2C_IDLE;
		}
	}

	/* Bit 7 ? DIF: Data Interrupt Flag */
	//J �f�[�^��M��������
	//J 1���������ACtrlB��CMD�ɓK���ȃR�}���h���������񂾂�t���O�N���A
	//J �N���A�����܂ŁASCL�̓z�[���h�����
	else if (status & TWI_SLAVE_DIF_bm) {
		uint8_t ack = I2C_SLAVE_RESPONCE_NACK;

		//J Master Read Operation
		if (sI2cSlaveMode == I2C_MASTER_READ) {
			/* Bit 4 ? RXACK: Received Acknowledge */
			//J NACK�̏ꍇ�A�����ʐM�͏I���
			if (status & TWI_SLAVE_RXACK_bm && !sIsFirstByte) {
				//J Nack������ƌ����āASlave���͉������邱�Ƃ͖����B���ʂ�����
				if (sSlaveContext.masterRxCB != NULL) {
					sSlaveContext.masterRxCB(I2C_SLAVE_RESPONCE_NACK);
				}

				I2C_SLAVE_CMD_COMPLETE(I2C_SLAVE_RESPONCE_ACK);
			}
			//J ACK�̏ꍇ�A���̃f�[�^���f�[�^���W�X�^�ɓ���ă��X�|���X��Ԃ�
			else {
				sIsFirstByte = 0;

				if (sSlaveContext.masterRxCB != NULL) {
					TWIC.SLAVE.DATA = sSlaveContext.masterRxCB(I2C_SLAVE_RESPONCE_ACK);
				} else {
					TWIC.SLAVE.DATA = 0x00;
				}

				I2C_SLAVE_CMD_RESPONSE(I2C_SLAVE_RESPONCE_ACK); // Clear DIF bit
			}
		}
		//J Master Write Operation
		else if (sI2cSlaveMode == I2C_MASTER_WRITE) {
			if (sSlaveContext.masterTxCB != NULL) {
				ack = sSlaveContext.masterTxCB(TWIC.SLAVE.DATA);
			}

			I2C_SLAVE_CMD_RESPONSE(ack); // Clear DIF bit
		}
	}

	return;
}

#ifdef SUPPORT_I2C_MASTER_INTERRUPT
/*---------------------------------------------------------------------------*/
//J I2C Master ������
ISR(TWIC_TWIM_vect)
{
	uint8_t ret = I2C_OK;
	uint8_t status = TWIC.MASTER.STATUS;

	//J Rx Done ������
	if (status & TWI_MASTER_RIF_bm) {
		//J ��M�J�E���^�̐������󂯎��
		sRxBuf[sRxCounter++] = TWIC.MASTER.DATA;

		//J �����o�b�t�@�������̂�NACK�Ԃ�
		if (sRxCounter >= sRxLen) {
			TWIC.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			volatile uint8_t wait = 10; //TODO
			while(wait--);
			ret = I2C_OK;
			goto CLEAN_UP;
		}
		//J ACK�Ԃ�
		else {
			TWIC.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
		}
	}
	//J Tx Done ������
	else if (status & TWI_MASTER_WIF_bm) {
		//J NACK�Ȃ�I���
		if (status & TWI_MASTER_ACKACT_bm) {
			ret = I2C_ERROR_NACK;
			goto CLEAN_UP;
		}
		//J ���M�f�[�^������Α��M����
		else if (sTxCounter < sTxLen) {
			TWIC.MASTER.DATA = sTxBuf[sTxCounter++];
		}
		//J ���M�f�[�^�������A��M���K�v�Ȃ��M����
		else if (sRxCounter < sRxLen) {
			I2C_RESTART();
			TWIC.MASTER.ADDR = ((sSlaveAddress << 1) | 1);
		}
		//J ���M�f�[�^���A��M�f�[�^���Ȃ����CB���ďI��
		else {
			ret = I2C_OK;
			goto CLEAN_UP;
		}
	}
	//J �o�X�ُ� ���� ���⎸�s�A
	else if (status & TWI_MASTER_ARBLOST_bm || status & TWI_MASTER_BUSERR_bm) {
		ret = I2C_BUS_ERROR;
		goto CLEAN_UP;
	}
	
	//J ����n�͂������Ŗ߂�
	return;

CLEAN_UP:
	I2C_STOP_CONDITION();
	DISABLE_I2C_MASTER_INTERRUPT();

	//J �e��ϐ��̃N���[���A�b�v
	sTxLen = 0;
	sRxLen = 0;
	sTxBuf = NULL;
	sRxBuf = NULL;
	sTxCounter = 0;
	sRxCounter = 0;
	
	sI2cMutex = 0;

	if (sI2cDoneCb) sI2cDoneCb(ret);

	return;
}

#endif/* SUPPORT_I2C_MASTER_INTERRUPT */

