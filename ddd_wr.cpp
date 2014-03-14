//------------------------------------------------------------------------------
//	Sets 3D3444 delay using TMB state machine
//
//	01/22/09 Initial
//------------------------------------------------------------------------------
// Chip 0:
// ch0	alct_tof_delay, shift entire ALCT in clockspace
// ch1	alct_txd_delay, replaced by digital phase shifter
// ch2	DMB tx clock
// ch3	RPC tx clock
//
// Chip 1:
// ch0:	alct_rxd_delay, replaced by digital phase shifter
// ch1:	mpc_delay
// ch2:	cfeb_tof_delay, shift all cfebs in clockspace
// ch3:	CFEB 0 clock
//
// Chip 2:
// ch0:	CFEB 1 clock
// ch1:	CFEB 2 clock
// ch2:	CFEB 3 clock
// ch3:	CFEB 4 clock
//
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
	long int	vme_write	(unsigned long &adr, unsigned short &wr_data);
	void		pause		(string s);

//------------------------------------------------------------------------------------------
// Entry ddd_wr(base_adr,ddd_chip,ddd_channel,ddd_delay)
//---------------------------------------------------------------------------------------------------------
	void ddd_wr(unsigned long &base_adr, const int &ddd_chip, const int &ddd_channel, const int &ddd_delay)
//---------------------------------------------------------------------------------------------------------
{
// VME addresses
	const unsigned long	vme_dddsm_adr		=0x000014;
	const unsigned long	vme_ddd0_adr		=0x000016;
	const unsigned long	vme_ddd1_adr		=0x000018;
	const unsigned long	vme_ddd2_adr		=0x00001A;
	const unsigned long	vme_dddoe_adr		=0x00001C;

// Local
	long			status;
	unsigned long	adr;
	unsigned short	rd_data;
	unsigned short	wr_data;

	int				autostart;
	int				i;
	int				ddd_busy;
	int				ddd_verify_ok;
	int				ddd_current;

//------------------------------------------------------------------------------
// Set new 3D3444 delay value for selected chip and channel
//------------------------------------------------------------------------------
// Check for invalid chip and channel
	if ((ddd_chip==0) && (ddd_channel==1)) pause ("ddd_wr: chip0 ch1 alct_txd_delay no longer used");
	if ((ddd_chip==1) && (ddd_channel==0)) pause ("ddd_wr: chip1 ch0 alct_rxd_delay no longer used");

// Get current 3D3444 delays for this chip
	adr	   = base_adr+vme_ddd0_adr+2*ddd_chip;
	status = vme_read(adr,rd_data);
	ddd_current = rd_data;

// Clean out old delay bits
	ddd_current = ddd_current & ~(0xF << (ddd_channel*4));

// Insert new delay
	wr_data = ddd_current | ((ddd_delay & 0xF) << (ddd_channel*4));
	status  = vme_write(adr,wr_data);

// Start DDD state machine
	adr	    = base_adr+vme_dddsm_adr;
	status  = vme_read(adr,rd_data);
	autostart = rd_data & 0x0020;	// get current autostart state
	wr_data	= 0x0000 | autostart;	// stop machine
	status	= vme_write(adr,wr_data);
	wr_data = 0x0001 | autostart;	// start machine
	status	= vme_write(adr,wr_data);
	wr_data = 0x0000 | autostart;	// unstart machine
	status	= vme_write(adr,wr_data);

// Wait for DDD to finish
	for (i=1; i<=1000; ++i) {		// this waits up to 16msec on my bit3
	status  = vme_read(adr,rd_data);
	ddd_busy      = (rd_data >> 6) & 0x1;
	ddd_verify_ok = (rd_data >> 7) & 0x1;
	if (ddd_busy==0) goto exit;
	} 
	printf("3d3444 verify failed %2i\n",ddd_delay);
	pause ("<cr>");

// Done
exit:
	return;
}
//------------------------------------------------------------------------------------------
// End ddd_wr
//------------------------------------------------------------------------------------------
