//------------------------------------------------------------------------------
//	Reads DCM Digital Phase Shift VME register
//
//	06/29/09 Copied from phaser_rd
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
	void		pause		(string s);

//------------------------------------------------------------------------------------------
// Function posneg_rd(base_adr, phaser_bank) returns posneg
//---------------------------------------------------------------------------------------------------------
	int posneg_rd(unsigned long &base_adr, const string phaser_bank)
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
	long			status;

// Local
	unsigned long	phaser_adr;
	int				posneg;

//------------------------------------------------------------------------------
// Read posneg value for selected DCM
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

// Get current phaser status
begin:
	adr		= base_adr+phaser_adr;
	status	= vme_read(adr,rd_data);
	posneg	= (rd_data >>  7) & 0x1;

	return posneg;
}
//------------------------------------------------------------------------------------------
// End posneg_rd
//------------------------------------------------------------------------------------------
