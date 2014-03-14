//------------------------------------------------------------------------------
//	Digital serial number single I/O cycle
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
	long int	vme_read	(unsigned long &adr, unsigned short &rd_data);
	long int	vme_write	(unsigned long &adr, unsigned short &wr_data);
	void		pause		(string s);

//------------------------------------------------------------------------------------------
// Entry dsn_io(adr,wr_data,rd_data)
//------------------------------------------------------------------------------------------
	void dsn_io (unsigned long &adr, unsigned short &wr_data, unsigned short &rd_data)
//------------------------------------------------------------------------------------------
{
	long int		status;
	unsigned short	wr_datax;
	int				busy_tmb;
	int				busy_mez;
	int				busy_rat;
	int				busy;
	int				nbusy;

// Write to the DSN state machine register
	nbusy  = 0;
	status = vme_write(adr,wr_data);	
rd:	status = vme_read (adr,rd_data);

	busy_tmb = (rd_data >>  3) & 0x1;
	busy_mez = (rd_data >>  8) & 0x1;
	busy_rat = (rd_data >> 13) & 0x1;
	busy     = busy_tmb | busy_mez | busy_rat;

// Wait for state machine to finish. 1ms worst case
	if (busy==0) goto done;
	nbusy++;
	if (nbusy<1000) goto rd;

	fprintf(stdout,"\tDSN state machine stuck busy. rd_data=%5.4X\n",rd_data);
	pause("<cr> to continue: ");

// End previous cycle
done:
	wr_datax = 0x0000;
	status   = vme_write(adr,wr_datax);

	return;
}
//------------------------------------------------------------------------------------------
// End dsn_io
//------------------------------------------------------------------------------------------
