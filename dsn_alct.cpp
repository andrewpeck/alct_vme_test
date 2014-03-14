//------------------------------------------------------------------------------
//	Read ALCT base board and mezzanine digital serial numbers via JTAG
//------------------------------------------------------------------------------
//	05/02/2012	Initial
//
//------------------------------------------------------------------------------
//	Headers
//------------------------------------------------------------------------------
	#define _CRT_SECURE_NO_WARNINGS 1
	#include <stdio.h>
	#include <iostream>
	#include <string>
	using namespace std;

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	#define			logical(L)		((L)?'T':'F')
	#define			yesno(L)		((L)?'y':'n')
	void			pause			(string s);
	void			stop			(string s);

	long int		vme_read		(unsigned long &adr, unsigned short &rd_data);
	long int		vme_write		(unsigned long &adr, unsigned short &wr_data);
	void			dow_crc			(int in[7], int &crc);

	void			vme_jtag_anystate_to_rti(unsigned long &adr, int &ichain);
	void			vme_jtag_write_ir		(unsigned long &adr, int &ichain, int &chip_id, int &opcode);
	void			vme_jtag_write_dr		(unsigned long &adr, int &ichain, int &chip_id, char wr_data[], char rd_data[], int &nbits);

//------------------------------------------------------------------------------
//	Entry
//------------------------------------------------------------------------------
	void dsn_alct (unsigned long &adr, int &ichain, int &chip_id, int &opcode);
{
// JTAG opcodes for ALCT DSN
	int				RdCfg         = 0x6;  // 	1	read	control register
	int				WrCfg         = 0x7;  // 	1	write	control register
	int				SNread        = 0x1B; //	1	read	one bit of serial number
	int				SNwrite0      = 0x1C; //	0	write	0 bit into serial number chip
	int				SNwrite1      = 0x1D; //	0	write	1 bit into serial number chip
	int				SNreset       = 0x1E; //	0	write	reset serial number chip

// Select base board or mezzanine

// Reset DS2401
	vme_jtag_anystate_to_rti(adr,ichain);
	vme_jtag_write_ir(adr,ichain,chip_id,SNreset );
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);

// Send read command 33h to ibutton chip
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);

	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);

// Read 64 bits of DSN bit by bit
	reg_len    = 1;											// Register length
	alct_sn[0] = 0;
	alct_sn[1] = 0;

	for (i=0; i<=63; ++i) {
	vme_jtag_write_ir(adr,ichain,chip_id,SNread);
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	ibit=tdo[0];
	if (i>= 0 && i < 31) alct_sn[0] = alct_sn[0] | (ibit<<i);
	if (i>=32 && i < 63) alct_sn[1] = alct_sn[1] | (ibit<<i);
	}

	printf("\n\tALCT DSN =%8.8X%8.8X\n",alct_sn[1],alct_sn[0]);

	alct_dsn_mfg = (alct_sn[0] >>  0) & 0x00FF;
	alct_dsn     = (alct_sn[0] >>  8) & 0xFFFFFF;
	alct_dsn_crc = (alct_sn[0] >> 28) & 0x00FF;

	printf("\n\tDigital Serial for ALCT");
	printf(" CRC=%2.2X",alct_dsn_crc);
	printf(" DSN=%6.6X",alct_dsn);
	printf(" MFG=%2.2X",alct_dsn_mfg);
	printf("\n");

// Check CRC
	dow_crc(dsn,icrc);
	icrc_ok="ERR!";

	if (icrc==dsn[7] && icrc!=0) icrc_ok = "OK  ";
	fprintf(stdout,"\tDigital Serial for %s CRC=%2.2X DSN=%",dsn_chip[itype].c_str(),dsn[7]);
	for (i=5; i>0; i--) fprintf(stdout,"%2.2X",dsn[i]);
	fprintf(stdout," MFG=%2.2X %s\n",dsn[0],icrc_ok.c_str());

	return;
}