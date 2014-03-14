//------------------------------------------------------------------------------
//	Sets posneg for selected DCM
//
//	06/29/09 Copied from phaser_wr
//	08/14/09 Add cfeb phaser banks
//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	long int	vme_read	(unsigned long &adr, unsigned short &rd_data);
	long int	vme_write	(unsigned long &adr, unsigned short &wr_data);
	void		pause		(string s);

//------------------------------------------------------------------------------------------
// Entry posneg_wr(base_adr, phaser_bank, posneg)
//---------------------------------------------------------------------------------------------------------
	void posneg_wr(unsigned long &base_adr, const string phaser_bank, const int &posneg)
//---------------------------------------------------------------------------------------------------------
{
// VME addresses
	const unsigned long	phaser0_adr = 0x00010E;
	const unsigned long	phaser1_adr = 0x000110;
	const unsigned long	phaser2_adr = 0x000112;
	const unsigned long	phaser3_adr = 0x000114;
	const unsigned long	phaser4_adr = 0x000116;
	const unsigned long	phaser5_adr = 0x000118;
	const unsigned long	phaser6_adr = 0x00011A;

// VME calls
	unsigned long	adr;
	unsigned short	rd_data;
	unsigned short	wr_data;
	long			status;

// Local
	unsigned long	phaser_adr;

//------------------------------------------------------------------------------
// Set new posneg value for selected DCM
//------------------------------------------------------------------------------
// Determine phaser bank VME address
	if		(phaser_bank.compare("alct_rxd"  )==0) {phaser_adr=phaser0_adr; goto begin;}
	else if	(phaser_bank.compare("alct_txd"  )==0) {phaser_adr=phaser1_adr; goto begin;}
	else if	(phaser_bank.compare("cfeb_rxd_0")==0) {phaser_adr=phaser2_adr; goto begin;}
	else if	(phaser_bank.compare("cfeb_rxd_1")==0) {phaser_adr=phaser3_adr; goto begin;}
	else if	(phaser_bank.compare("cfeb_rxd_2")==0) {phaser_adr=phaser4_adr; goto begin;}
	else if	(phaser_bank.compare("cfeb_rxd_3")==0) {phaser_adr=phaser5_adr; goto begin;}
	else if	(phaser_bank.compare("cfeb_rxd_4")==0) {phaser_adr=phaser6_adr; goto begin;}
	else	{printf("\nPhaser bank unknown: %s",phaser_bank.c_str()); pause ("<cr>");}

// Get current phaser register
begin:
	adr		= base_adr+phaser_adr;
	status	= vme_read(adr,rd_data);

// Write new posneg
	wr_data = rd_data & ~(1 << 7);
	wr_data = wr_data | (posneg << 7);
	status	= vme_write(adr,wr_data);

	return;
}
//------------------------------------------------------------------------------------------
// End posneg_wr
//------------------------------------------------------------------------------------------
