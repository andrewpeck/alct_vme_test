//------------------------------------------------------------------------------
//	Reads DCM Digital Phase Shift VME register
//
//	06/15/09 Initial
//	06/29/09 Added posneg bit
//	06/30/09 Add phase delta
//	08/14/09 Add cfeb phaser banks
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
	long int	vme_read	(unsigned long &adr, unsigned short &rd_data);
	void		pause		(string s);

//------------------------------------------------------------------------------
// Function phaser_rd(base_adr, phaser_bank, phaser_delta) returns phaser_delay
//------------------------------------------------------------------------------
	int phaser_rd(unsigned long &base_adr, const string phaser_bank, const int &phaser_delta)
//------------------------------------------------------------------------------
//	base_adr     = VME base address for this TMB
//	phaser_bank  = "alct_txd" or "alct_rxd"
//	phaser_delta = scale factor for phaser_delay
//	returns		 = current phaser_delay=0-to-255 clock phase
//------------------------------------------------------------------------------
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
	int				phaser_delay;

// Phaser register
	int				fire;
	int				reset;
	int				busy;
	int				lock;
	int				sm_vec;
	int				posneg;
	int				phase;
	int				qcycle;
	int				hcycle;

// Phaser machine states
	const string	sm_dsp[8] = {
	"init     ",
	"wait_tmb ",
	"wait_dcm ",
	"init_dps ",
	"inc_dec  ",
	"wait_dps ",
	"unfire   ",
	"undefined"
	};

//------------------------------------------------------------------------------
// Read phase delay value for selected DCM
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
	fire	= (rd_data >>  0) & 0x1;
	reset	= (rd_data >>  1) & 0x1;
	busy	= (rd_data >>  2) & 0x1;
	lock	= (rd_data >>  3) & 0x1;
	sm_vec	= (rd_data >>  4) & 0x7;
	posneg	= (rd_data >>  7) & 0x1;
	phase	= (rd_data >>  8) & 0x3F;
	qcycle	= (rd_data >> 14) & 0x1;
	hcycle	= (rd_data >> 15) & 0x1;

	phaser_delay = phase | (qcycle << 6) | (hcycle << 7);

// Display status
	dprintf(log_file,"\n");
	dprintf(log_file,"Phaser Call: phaser_bank=%s phaser_delay=%3i phaser_delta=%3i\n",phaser_bank.c_str(),phaser_delay, phaser_delta);
	dprintf(log_file,"fire   = %3i\n",fire);
	dprintf(log_file,"reset  = %3i\n",reset);
	dprintf(log_file,"busy   = %3i\n",busy);
	dprintf(log_file,"lock   = %3i\n",lock);
	dprintf(log_file,"sm_vec = %3i %s\n",sm_vec,sm_dsp[sm_vec].c_str());
	dprintf(log_file,"posneg = %3i\n",posneg);
	dprintf(log_file,"phase  = %3i\n",phase);
	dprintf(log_file,"qcycle = %3i\n",qcycle);
	dprintf(log_file,"hcycle = %3i\n",hcycle);
	dprintf(log_file,"\n");

// Beam me up, Scotty
	return phaser_delay/phaser_delta;
}
//------------------------------------------------------------------------------------------
// End phaser_rd
//------------------------------------------------------------------------------------------
