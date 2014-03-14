//------------------------------------------------------------------------------
// JTAG IO Byte
//
// Clocks tck for nframes number of cycles.
//	Writes nframes of data to tms and tdi on the falling edge of tck.
//	Reads tdo on the rising clock edge.
//
//	Caller passes tms and tdi byte arrays with 1 bit per byte.	
//	Returned data is also 1 bit per byte. Inefficent,but easy.
//
//	tms()	=	byte's lsb to write to parallel port
//	tdi()	=	byte's lsb to write to parallel port
//	tdo()	=	bit read back from parallel port, stored in lsb
//	tck		=	toggled by this routine
//
//	03/13/00 Initial
//	10/26/01 Adapted for Xilinx X-blaster
//	12/10/01 Adapted for TMB VME boot register
//  03/08/06 Added debug frame capture for prom images
//	10/09/08 Port from fortran
//	11/13/08 Fix tdo
//	10/07/08 Port from fortran version
//	11/13/08 Update for vmetst calls
//	03/03/10 Add logging common
//	09/08/10 Missing semicolon caused final read to be skipped
//	07/11/2012	Add jtag verilog output for simulator
//
//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
//	#define debug_jtag_iobyte 1
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------
// Common
//------------------------------------------------------------------------------
	extern FILE *log_file;

// Common/jtag_common/
	extern FILE *jtag_file;
	extern bool	jtaglogmode;

//------------------------------------------------------------------------------
//	Debug print mode
//------------------------------------------------------------------------------
//	#define debug_jtag_io_byte 1	// comment this line to turn off debug print

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------
	long int		vme_read		(unsigned long &adr, unsigned short &rd_data);
	long int		vme_write		(unsigned long &adr, unsigned short &wr_data);

	void			pause			(string s);
	void			step			(int &tck_bit, int &tms_bit, int &tdi_bit, int &tdo_bit);

//--------------------------------------------------------------------------------------------------------------------------------------
	void vme_jtag_io_byte (unsigned long &adr, int &ichain, int &nframes, char tms[], char tdi[], char tdo[], const int &step_mode)
//--------------------------------------------------------------------------------------------------------------------------------------
{
	const int		mxframes=1000;
	unsigned short	rd_data;
	unsigned short	wr_data;

	int				status;
	int				boot_state;

	int				jtag_word;
	int				jtag_out;
	int				jtag_in;

	int				jtag_en;
	int				sel0,sel1,sel2,sel3;
	int				iframe;

	int				tck_bit;
	int				tms_bit;
	int				tdi_bit;

	int				tck_i4;
	int				tms_i4;
	int				tdi_i4;
	int				tdo_i4;

//------------------------------------------------------------------------------
// Get current boot register
	status     = vme_read(adr,rd_data);
	boot_state = rd_data & 0x7F80;

// Set tck,tms,tdi low, select jtag chain, enable VME control of chain
	tck_bit = 0;									//TCK low
	tms_bit = 0;									//TMS low
	tdi_bit = 0;									//TDI low
	jtag_en = 1;									//Boot register sources JTAG

	sel0 = (ichain >> 0) & 0x1;						//JTAG chain select
	sel1 = (ichain >> 1) & 0x1;
	sel2 = (ichain >> 2) & 0x1;
	sel3 = (ichain >> 3) & 0x1;

	jtag_word = (boot_state & 0x7FF8) |				//Clear jtag bits
	(tdi_bit << 0) |
	(tms_bit << 1) |
	(tck_bit << 2) |
	(sel0    << 3) |
	(sel1    << 4) |
	(sel2    << 5) |
	(sel3    << 6) |
	(jtag_en << 7);

	wr_data = jtag_word;
	status  = vme_write(adr,wr_data);				//Write boot reg
	if (jtaglogmode) fprintf(jtag_file,"\t#50 tck=%1i; tms=%1i; tdi=%1i;\n",tck_bit!=0,tms_bit!=0,tdi_bit!=0);

// Loop over jtag frames
	if  (nframes<=0) goto done;						//No frames to send
	for (iframe=0; iframe<nframes; ++iframe) {		//Loop over input data frames
	tdo[iframe] = 0;								//Clr tdo 

	tck_i4 = 0;										//Take TCK low
	tms_i4 = tms[iframe];							//Convert byte to I4
	tdi_i4 = tdi[iframe];							//Convert byte to I4

	tck_bit = (tck_i4 << 2);						//TCK bit
	tms_bit = (tms_i4 << 1);						//TMS bit
	tdi_bit = (tdi_i4 << 0);						//TDI bit

	jtag_out = jtag_word & 0x7FF8;					//Clear old state
	jtag_out = jtag_out | tck_bit | tms_bit | tdi_bit;
	wr_data  = jtag_out;
	status   = vme_write(adr,wr_data);				//Write boot reg
	if (jtaglogmode) fprintf(jtag_file,"\t#50 tck=%1i; tms=%1i; tdi=%1i;\n",tck_bit!=0,tms_bit!=0,tdi_bit!=0);

	status  = vme_read(adr,rd_data);				//Read boot reg
	jtag_in = rd_data;
	tdo_i4  = (jtag_in >> 15);						//Extract tdo bit
	tdo_i4  = tdo_i4 & 0x1;							//Mask lsb
	if (step_mode!=0) step(tck_bit,tms_i4,tdi_i4,tdo_i4);

	tck_i4   = 1;									//Take TCK high, leave tms,tdi as they were
	tck_bit  = (tck_i4 << 2);						//Take TCK high, leave tms,tdi as they were
	jtag_out = jtag_out | tck_bit | tms_bit | tdi_bit;
	wr_data  = jtag_out;
	status   = vme_write(adr,wr_data);				//Write boot reg
	if (jtaglogmode) fprintf(jtag_file,"\t#50 tck=%1i; tms=%1i; tdi=%1i;\n",tck_bit!=0,tms_bit!=0,tdi_bit!=0);

	status  = vme_read(adr,rd_data);				//Read boot reg
	jtag_in = rd_data;
	tdo_i4  = (jtag_in >> 15);						//Extract tdo bit
	tdo_i4  = tdo_i4 & 0x1;							//Mask lsb
	tdo[iframe] = tdo_i4;							//Convert I4 to byte for caller

	if (jtaglogmode) fprintf(jtag_file,"\t#0  tdo_in=%1i;\n",tdo_i4);
	if (step_mode!=0) step(tck_bit,tms_i4,tdi_i4,tdo_i4);
	}												//Close for iframe

// Put JTAG bits into idle state
done:
	jtag_out = jtag_out & 0xFFFB;					//Take TCK low, leave others as they were
	wr_data  = jtag_out;
	status   = vme_write(adr,wr_data);				//Write boot reg
	if (jtaglogmode) fprintf(jtag_file,"\t#50 tck=%1i; tms=%1i; tdi=%1i;\n",0,0,0);

// Debug print
	#ifdef debug_jtag_io_byte
	int i;
	fprintf(log_file,"jtag_io_byte: adr=%6X ichain=%1X nframes=%5i\n",adr,ichain,nframes);
	fprintf(log_file,"jtag_io_byte: tms="); for (i=0;i<nframes; ++i) {fprintf(log_file,"%1X",tms[i]);} fprintf(log_file,"\n");
	fprintf(log_file,"jtag_io_byte: tdi="); for (i=0;i<nframes; ++i) {fprintf(log_file,"%1X",tdi[i]);} fprintf(log_file,"\n");
	fprintf(log_file,"jtag_io_byte: tdo="); for (i=0;i<nframes; ++i) {fprintf(log_file,"%1X",tdo[i]);} fprintf(log_file,"\n");
	#endif
	return;
}
//------------------------------------------------------------------------------
	void step (int &tck_bit, int &tms_bit, int &tdi_bit, int &tdo_bit)
//------------------------------------------------------------------------------
{
	int tck_cvt=0;
	if (tck_bit != 0) tck_cvt=1;

	printf("tck=%1i tms=%1i tdi=%1i tdo=%1i paused: ",tck_cvt,tms_bit,tdi_bit,tdo_bit);
	char unpause=getchar();
	return;
}
//------------------------------------------------------------------------------
// End jtag_io_byte
//------------------------------------------------------------------------------
