//------------------------------------------------------------------------------
//	Read TMB ADC
//
//	Device is Texas Instruments TLC2543CDW
//
//	02/08/06 Ported from vmetst_v5
//	12/11/08 Port from vmetst_v7 fortran to c++
//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------
// Debug print mode
//------------------------------------------------------------------------------
//	#define debug 1	// comment this line to turn off debug print

	#ifdef debug
	 #define dprintf fprintf
	#else
	 #define dprintf //
	#endif

//------------------------------------------------------------------------------
// Common
//------------------------------------------------------------------------------
	extern FILE *log_file;

// Common/adc_common/
	extern double		adc_voltage[14];
	extern double		v5p0;
	extern double		v3p3;
	extern double		v1p5core;
	extern double		vcore_expect;
	extern double		vcore_noload;
	extern double		v1p5tt;
	extern double		v1p0;
	extern double		v1p0_expect;
	extern double		a5p0;
	extern double		a3p3;
	extern double		a1p5core;
	extern double		a1p5tt;
	extern double		a3p3rat;
	extern double		a1p8rat;
	extern double		v3p3rat;
	extern double		v1p8rat;
	extern double		vref2;
	extern double		vzero;
	extern double		vref;

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------
	long int		vme_read	(unsigned long &adr, unsigned short &rd_data);
	long int		vme_write	(unsigned long &adr, unsigned short &wr_data);

//------------------------------------------------------------------------------
// Entry adc_read()
//------------------------------------------------------------------------------
	void			adc_read (unsigned long &base_adr)
//------------------------------------------------------------------------------
{
// VME address
	const unsigned long	vme_adc_adr = 0x000024;

// Local
	long			status;
	unsigned long	adr;
	unsigned short	rd_data;
	unsigned short	wr_data;

	int				ich;
	int				adc_dout;
	int				adc_sclock;
	int				adc_din;
	int				adc_cs;
	int				adc_shiftin;
	int				adc_counts[14];
	int				iclk;

//------------------------------------------------------------------------------
//	Read TMB ADC Channels
//------------------------------------------------------------------------------
// Loop over 14 ADC channels of 12 adc clock cycles each, plus 1 more cycle to read last channel
	adr = vme_adc_adr+base_adr;

	for (ich=0; ich<=14; ++ich) {
	adc_dout    = 0;
	adc_shiftin = (ich << 4);		// d[7:4]=channel, d[3:2]=length, d[1:0]=ldbf,bip
	if(ich >= 0xE) adc_shiftin = 0; // Don't send channel 14, it is power-down

// Take ADC cs high, clock low
	adc_sclock	= 0;
	adc_din		= 0;
	adc_cs		= 1;

	wr_data = 0x0000;
	wr_data = wr_data | (adc_sclock << 6);
	wr_data = wr_data | (adc_din    << 7);
	wr_data = wr_data | (adc_cs     << 8);
	status  = vme_write(adr,wr_data);

// Loop over 12 adc clocks, for 12-bit, unipolar format
	for (iclk=0; iclk<=11; ++iclk) {

// Clock low: output channel number for next cycle, msb first		
	if (iclk <= 7)	adc_din = (adc_shiftin >> (7-iclk)) & 0x1;
	else			adc_din = 0;

	adc_sclock	= 0;
	adc_cs		= 0;

	wr_data = 0x0000;
	wr_data = wr_data | (adc_sclock << 6);
	wr_data = wr_data | (adc_din    << 7);
	wr_data = wr_data | (adc_cs     << 8);
	status  = vme_write(adr,wr_data);

// Clock high: send channel number for next cycle
	adc_sclock	= 1;
	adc_cs		= 0;

	wr_data = 0x0000;
	wr_data = wr_data | (adc_sclock << 6);
	wr_data = wr_data | (adc_din    << 7);
	wr_data = wr_data | (adc_cs     << 8);
	status  = vme_write(adr,wr_data);

// Clock high: read ADC data for previous channel, msb first
	status   = vme_read(adr,rd_data);
	rd_data  = (rd_data >> 5) & 0x1;
	adc_dout = adc_dout | (rd_data << (11-iclk));
	}	// close iclk

// Take ADC cs high, clock low
	adc_sclock	= 0;
	adc_din		= 0;
	adc_cs		= 1;

	wr_data = 0x0000;
	wr_data = wr_data | (adc_sclock >> 6);
	wr_data = wr_data | (adc_din    >> 7);
	wr_data = wr_data | (adc_cs     >> 8);
	status  = vme_write(adr,wr_data);

// Close channel loop
	if (ich >= 1) { 
	adc_counts[ich-1]  = adc_dout;
	adc_voltage[ich-1] =(float(adc_dout)/float(4095))*4.095;
	dprintf(stdout,"\tCh=%1X data=%8.8X %6.3f\n",ich-1,adc_dout,adc_voltage[ich-1]);
	}	// close if  ich
	}	// close for ich

// Convert ADC counts to voltages and currents
	v5p0	 = adc_voltage[ 0]*2.0;	// 1v/2v
	v3p3	 = adc_voltage[ 1];
	v1p5core = adc_voltage[ 2];
	v1p5tt	 = adc_voltage[ 3];
	v1p0	 = adc_voltage[ 4];
	a5p0	 = adc_voltage[ 5]/0.2;	// 200mv/Amp
	a3p3	 = adc_voltage[ 6]/0.2;
	a1p5core = adc_voltage[ 7]/0.2;
	a1p5tt	 = adc_voltage[ 8]/0.2;
	a1p8rat	 = adc_voltage[ 9]/0.2;	// if SH921 set 1-2
	v3p3rat	 = adc_voltage[ 9];		// if SH921 set 2-3
	v1p8rat  = adc_voltage[10];

	vref2=adc_voltage[11];
	vzero=adc_voltage[12];
	vref= adc_voltage[13];

	return;
}
