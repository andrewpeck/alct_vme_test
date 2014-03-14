//------------------------------------------------------------------------------
//	Calls dsn_io_alct to read ALCT digital serial numbers
//
//	01/04/2002	Initial
//	11/13/2008	Port to c++
//	05/02/2012	Copy from TMB version
//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	void dsn_io_alct (unsigned long &adr, int &ichain, int &chip_id, int &opcode_rd, int &opcode_wr, int &reg_len, unsigned short &wr_data, unsigned short &rd_data);

//------------------------------------------------------------------------------------------
// Entry dsn_rd(vme_dsn_adr,itype,dsn)
//------------------------------------------------------------------------------------------
	void dsn_rd_alct (unsigned long &adr, int &ichain, int &chip_id, int &opcode_rd, int &opcode_wr, int &reg_len, const int &itype, int dsn[])
//------------------------------------------------------------------------------------------
{
	unsigned short	wr_data;
	unsigned short	rd_data;

	int				ioffset;
	int				ibit;
	int				i;
	int				j;
	int				dsn_bit[64];
	int				idata;

// VME dsn register offset for mez,0=TMB dsn, 1=Mezzanine dsn
	ioffset = itype*5;

// Send Init pulse >480us
	wr_data = 0x0005;
	wr_data = (wr_data << ioffset);
	dsn_io_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len,wr_data,rd_data);

// Send ROM Read command 33h
	for (i=0; i<=7; i++) {
	ibit    = (0x33 >> i) & 0x1;
	wr_data = 0x0001 | (ibit << 1);
	wr_data = (wr_data << ioffset);
	dsn_io_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len,wr_data,rd_data);
	}

// Read 64 bits of ROM data
	for (i=0; i<=63; i++) {
	wr_data = 0x0003;
	wr_data = (wr_data << ioffset);
	dsn_io_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len,wr_data,rd_data);
	dsn_bit[i] = (rd_data >> (4+ioffset)) & 0x1;
	}

// Pack 64 bits into array of 8 bytes
	for (j=0; j<=7; j++) {
	idata = 0;
	for (i=0; i<=7; i++) {
	ibit = i+8*j;
	idata=idata | (dsn_bit[ibit] << i);
	dsn[j]=idata;
	}}

	return;
}
//------------------------------------------------------------------------------------------
// End dsn_rd
//------------------------------------------------------------------------------------------
