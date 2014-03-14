//------------------------------------------------------------------------------
//	JTAG register read/write that looks like the VME version
//
//	05/03/2012	Initial
//
//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	#include <string>
	using namespace std;

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
// JTAG register read that looks like the VME version
//------------------------------------------------------------------------------------------
	long int jtag_read (unsigned long &adr, int &ichain, int &chip_id, int &opcode, int &reg_len, unsigned short &rd_data)
{
	char tdi[32]={0};
	char tdo[32]={0};
	long i4;

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read register contents

	tdi_to_i4(&tdo[0],i4,reg_len,0);						// Convert tdo to integer				

	rd_data = (unsigned short)i4;

	return 0;
}

//------------------------------------------------------------------------------------------
// JTAG register write that looks like the VME version
//------------------------------------------------------------------------------------------
	long int jtag_write	(unsigned long &adr, int &ichain, int &chip_id, int &opcode, int &reg_len, unsigned short &wr_data)
{
	char tdi[32]={0};
	char tdo[32]={0};
	long i4;

	i4_to_tdi(i4=wr_data,&tdi[0],reg_len,0);				// Convert integer to tdi bit array

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write register contents

	return 0;
}
//------------------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------------------
