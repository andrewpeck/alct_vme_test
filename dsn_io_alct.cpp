//------------------------------------------------------------------------------
//	Digital serial number single I/O cycle for ALCT
//
//	01/04/2002	Initial
//	11/13/2008	Port to c++
//	05/02/2012	Port to ALCT version
//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------
// Common
//------------------------------------------------------------------------------
	extern FILE *log_file;

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	void		vme_jtag_anystate_to_rti(unsigned long &adr, int &ichain);
	void		vme_jtag_write_ir		(unsigned long &adr, int &ichain, int &chip_id, int &opcode);
	void		vme_jtag_write_dr		(unsigned long &adr, int &ichain, int &chip_id, char wr_data[], char rd_data[], int &nbits);

	long int	jtag_read				(unsigned long &adr, int &ichain, int &chip_id, int &opcode, int &reg_len, unsigned short &rd_data);
	long int	jtag_write				(unsigned long &adr, int &ichain, int &chip_id, int &opcode, int &reg_len, unsigned short &wr_data);

	void		i4_to_tdi				(long int &i4, char  tdi[], const int &nbits, const int &spi);
	void		tdi_to_i4				(char  tdi[], long int &i4, const int &nbits, const int &spi);
	void		pause					(string s);

//------------------------------------------------------------------------------------------
// ALCT JTAG register definitions
//------------------------------------------------------------------------------------------
//	dsn_wr_reg[0]	alct_sn_start	In	Begin counting
//	dsn_wr_reg[1]	alct_sn_write	In	DSN data bit to output to chip
//	dsn_wr_reg[2]	alct_sn_init	In	DSN init mode
//
//	dsn_wr_reg[5]	mez_sn_start	In	Begin counting
//	dsn_wr_reg[6]	mez_sn_write	In	DSN data bit to output to chip
//	dsn_wr_reg[7]	mez_sn_init		In	DSN init mode

//	dsn_rd_reg[3]	alct_sn_busy	Out	DSN chip is busy
//	dsn_rd_reg[4]	alct_sn_data	Out	DSN data read 

//	dsn_rd_reg[8]	mez_sn_busy		Out	DSN chip is busy
//	dsn_rd_reg[9]	mez_sn_data		Out	DSN data read 

//------------------------------------------------------------------------------------------
// Entry dsn_io_alct(adr,wr_data,rd_data)
//------------------------------------------------------------------------------------------
	void dsn_io_alct (unsigned long &adr, int &ichain, int &chip_id, int &opcode_rd, int &opcode_wr, int &reg_len, unsigned short &wr_data, unsigned short &rd_data)
//------------------------------------------------------------------------------------------
{
// Local
	long int		status;
	unsigned short	wr_datax;
	int				busy_alct;
	int				busy_mez;
	int				busy;
	int				nbusy;

// Write to the DSN state machine register
	nbusy  = 0;
	vme_jtag_anystate_to_rti(adr,ichain);			// Take TAP to RTI
	status = jtag_write(adr,ichain,chip_id,opcode_wr,reg_len,wr_data);	
rd:	status = jtag_read (adr,ichain,chip_id,opcode_rd,reg_len,rd_data);

	busy_alct = (rd_data >>  3) & 0x1;
	busy_mez  = (rd_data >>  8) & 0x1;
	busy      = busy_alct | busy_mez;

// Wait for state machine to finish. 1ms worst case
	if (busy==0) goto done;
	nbusy++;
	if (nbusy<1000) goto rd;

	printf("\tDSN state machine stuck busy. rd_data=%5.4X\n",rd_data);
	pause("<cr> to continue: ");

// End previous cycle
done:
	wr_datax = 0x0000;
	status   = jtag_write(adr,ichain,chip_id,opcode_wr,reg_len,wr_datax);
	vme_jtag_anystate_to_rti(adr,ichain);			// Take TAP to RTI
	return;
}

//------------------------------------------------------------------------------------------
// End dsn_io
//------------------------------------------------------------------------------------------
