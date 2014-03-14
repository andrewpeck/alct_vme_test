//------------------------------------------------------------------------------
//	Calls dsn_io to read TMB digital serial numbers
//
//	01/04/02 Initial
//	11/13/08 Port to c++
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
//	Debug print mode
//------------------------------------------------------------------------------
//	#define debug 1	// comment this line to turn off debug print

	#ifdef debug
	 #define dprintf fprintf
	#else
	 #define dprintf //
	#endif

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	void dsn_io (unsigned long &adr, unsigned short &wr_data, unsigned short &rd_data);

//------------------------------------------------------------------------------------------
// Entry dsn_rd(vme_dsn_adr,itype,dsn)
//------------------------------------------------------------------------------------------
	void dsn_rd(unsigned long &vme_dsn_adr, const int &itype, int dsn[])
//------------------------------------------------------------------------------------------
{
	unsigned long	adr;
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
	adr     = vme_dsn_adr;
	wr_data = 0x0005;
	wr_data = (wr_data << ioffset);
	dsn_io(adr,wr_data,rd_data);

// Send ROM Read command 33h
	for (i=0; i<=7; i++) {
	ibit    = (0x33 >> i) & 0x1;
	wr_data = 0x0001 | (ibit << 1);
	wr_data = (wr_data << ioffset);
	dsn_io(adr,wr_data,rd_data);
	}

// Read 64 bits of ROM data
	for (i=0; i<=63; i++) {
	wr_data = 0x0003;
	wr_data = (wr_data << ioffset);
	dsn_io(adr,wr_data,rd_data);
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
