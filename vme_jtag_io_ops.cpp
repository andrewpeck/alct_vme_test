//------------------------------------------------------------------------------
//	JTAG IO Operations
//
//	10/06/2008	Port from fortran version
//	12/13/2010	Add jtag cable detection
//	04/16/2012	Add ALCT slow control fpga+prom chain
//	04/24/2012	Restructure to separate chain address from chain type
//	06/21/2012	Add ALCT slow control user chain
//------------------------------------------------------------------------------
//	Headers
//------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------
//	Common
//------------------------------------------------------------------------------
	extern FILE *log_file;

//  VME Chain	Function
//  ---------	------------------------------------
//	0x0	0000	ALCT Slow control User
//	0x1	0001	ALCT Slow control FPGA+PROM
//	0x2	0010	ALCT Mezzanine    User
//	0x3	0011	ALCT Mezzanine    FPGA+PROM		+ Variants for Virtex-E and Spartan-6
//	0x4	01XX	TMB  Mezzanine    FPGA+PROM		+ Variants for Virtex-2 and Virtex-6
//	0x8	10XX	TMB  User         PROMs
//	0xC	1100	FPGA Monitor      For TMB self-test
//	0xD	1101	RAT  Module       FPGA+PROM

	const int	mxchain   = 9;									// Chain_id
	const int	mxdevices = 5;									// Devices in chain

	int			devices_in_chain[mxchain]={2,2,2,5,3,2,3,3,1};	// Chain 0,1,2,3,4,5,6,7,8
	int			bits_per_opcode[mxchain][mxdevices]={
				{ 8, 8, 0, 0, 0},		// 0 TMB user prom chain		PROM, PROM
				{ 5, 8, 0, 0, 0},		// 1 RAT chain					FPGA, PROM
				{ 5, 8, 0, 0, 0},		// 2 ALCT VirtexE  mez chain	FPGA, PROM
				{ 6, 8, 8, 8, 8},		// 3 TMB  Virtex2  mez chain	FPGA, PROM, PROM, PROM, PROM
				{10,16,16, 0, 0},		// 4 TMB  Virtex6  mez chain	FPGA, PROM, PROM
				{ 3, 8, 0, 0, 0},		// 5 ALCT Slow control chain	FPGA, PROM
				{ 6,16,16, 0, 0},		// 6 ALCT Spartan6 mez chain	FPGA, PROM, PROM
				{ 6,16,16, 0, 0},		// 7 TMB  Spartan6 mez chain	FPGA, PROM, PROM
				{ 6, 0, 0, 0, 0}};		// 8 ALCT Slow control chain	User JTAG

//------------------------------------------------------------------------------
//	Debug print mode
//------------------------------------------------------------------------------
//	#define debug_jtag_io_ops 1	// comment this line to turn off debug print

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	void vme_jtag_io_byte	(unsigned long &adr, int &ichain, int &nframes, char tms[], char tdi[], char tdo[], const int &step_mode);
	void pause				(string s);

//------------------------------------------------------------------------------
//	Convert integer*4 word to 1 bit per byte for JTAG transmission
//	SPI=0 for FPGAs and PROMs
//	SPI=1 for DACs that take msb first
//------------------------------------------------------------------------------
	void i4_to_tdi(long int &i4, char tdi[], const int &nbits, const int &spi)
{
	long int	i4_copy;
	int			ibit;

	if (nbits > 32) pause("i4_to_tdi: nbits>32 wtf?");

	if (spi==0) {				// Translate lsb first
	i4_copy=i4;
	for (ibit=0; ibit<nbits; ++ibit) {
	tdi[ibit]=i4_copy & 0x1;
	i4_copy=(i4_copy >> 1);
	}}

	else {						// Translate msb first
	i4_copy=i4;
	for (ibit=0; ibit<nbits; ++ibit) {
	tdi[nbits-1-ibit]=i4_copy & 0x1;
	i4_copy=(i4_copy >> 1);
	}}

	#ifdef debug_jtag_io_ops
	fprintf(log_file,"i4_to_tdi: i4=%8.8X nbits=%4i spi=%1i tdi=",i4,nbits,spi); 
	for (int i=0; i<nbits; ++i) fprintf(log_file,"%1i",tdi[i]); fprintf(log_file,"\n");
	#endif

	return;
}
//------------------------------------------------------------------------------
//	Convert tdi of 1 bit per byte into an integer*4 word
//	SPI=0 for FPGAs and PROMs
//	SPI=1 for DACs that take msb first
//------------------------------------------------------------------------------
	void tdi_to_i4(char tdi[], long int &i4, const int &nbits,  const int &spi)
{
	long int	i4_copy;
	int			ibit;
	int			bit;

	if (nbits > 32) pause("tdi_to_i4: nbits>32 wtf?");

	if(spi==0) {				// Translate lsb first
	i4_copy=0;
	for (ibit=0; ibit<nbits; ++ibit) {
	bit=tdi[ibit] & 0x1;
	i4_copy=i4_copy | (bit*(1 << ibit));
	}}

	else {						// Translate msb first
	i4_copy=0;
	for (ibit=0; ibit<nbits; ++ibit) {
	bit=tdi[nbits-1-ibit] & 0x1;
	i4_copy=i4_copy | bit*(1 << ibit);
	}}

	i4=i4_copy;

	#ifdef debug_jtag_io_ops
	fprintf(log_file,"tdi_to_i4: i4=%8.8X nbits=%4i spi=%1i tdi=",i4,nbits,spi); 
	for (int i=0; i<nbits; ++i) fprintf(log_file,"%1i",tdi[i]); fprintf(log_file,"\n");
	#endif

	return;
}
//------------------------------------------------------------------------------
//	Take jtag TAP controller from any state to RTI
//------------------------------------------------------------------------------
	void vme_jtag_anystate_to_rti(unsigned long &adr, int &ichain)
{
	const int	mxbits=6;							// # bits of tms

	char		tms[mxbits];
	char		tdi[mxbits];
	char		tdo[mxbits];

	char		tms_rti[mxbits]={1,1,1,1,1,0};		// Anystate to TLR then RTI
	char		tdi_rti[mxbits]={0,0,0,0,0,0};

	int			iframe;
	int			nframes;
	int			ibit;
	int			step_mode;

//	Take JTAG tap from any state to TLR then to RTI
	iframe = 0;										// Jtag frame number

	for (ibit=0; ibit<mxbits; ++ibit) {				// Go from any state to RTI
	tms[iframe] = tms_rti[ibit];					// Take tap to RTI
	tdi[iframe] = tdi_rti[ibit];
	iframe++;										// Next frame number
	}

	step_mode = 0;									// 1=single step, 0=run
	nframes   = iframe;								// Total frames to send
	vme_jtag_io_byte(adr,ichain,nframes,tms,tdi,tdo,step_mode);
	return;
}
//-----------------------------------------------------------------------------------------------
//	Write opcode to the JTAG Instruction Register for the specified chip
//
//	All chips except current chip are set to bypass mode.
//	It is presumed that the TAP controllers are presently in the RTI state
//
//	JTAG instructions and data are loaded for
//	PROM1 first and PROM0 last:
//
//	JTAG				Load	TMB		RAT
//	Position Chip ID	Order	Name	Name
//	tdi>0		0		1		PROM0	FPGA
//	tdo<1		1		0		PROM1	PROM
//
//	05/08/2000	Changed tms_post_opcode to 2 TCKs instead of 3
//	10/27/2001	Adapted for TMB2001A Xilinx User PROMs
//	06/02/2004	Modified for multiple chains to support RAT and TMB
//	06/03/2004	Modified for TMB2005a mezzanine chain
//	10/09/2008	Convert to c++
//	06/10/2011	Add Virtex-6
//	04/16/2012	Add Spartan-6 and ALCT slow control PROM
//	04/24/2012	Restructure to separate chain address from chain type
//------------------------------------------------------------------------------
	void vme_jtag_write_ir(unsigned long &adr, int &ichain, int &chip_id, int &opcode)
{
	const int	mxbits = 128;							// Maximum # bits of tms
	char		tms[mxbits];
	char		tdi[mxbits];
	char		tdo[mxbits];

	char		tms_pre_opcode[4]={1,1,0,0};			// RTI to ShfIR
	char		tdi_pre_opcode[4]={0,0,0,0};

	char		tms_post_opcode[2]={1,0};				// Ex1IR to RTI
	char		tdi_post_opcode[2]={0,0};

	char		prom_ir[8]={1,0,0,2,2,0,0,0};			// 2=don't care flag

	int			chain_id;
	int			iframe;
	int			nframes;
	int			idevice;
	int			ichip;
	int			ibit;
	int			bit;
	int			preamble;
	int			postamble;
	int			wr_opcode;
	int			iopbit;
	int			ndevices;
	int			nbitsop;
	int			step_mode;
	int			i;
//------------------------------------------------------------------------------
// Set up TMS to take TAP from RTI to Shift IR
	chain_id=-1;
	if (ichain==0x08) chain_id = 0;						// TMB user prom chain
	if (ichain==0x0C) chain_id = 0;						// TMB fpga chain
	if (ichain==0x0D) chain_id = 1;						// RAT chain
	if (ichain==0x02) chain_id = 2;						// ALCT mez  cfg chain
	if (ichain==0x03) chain_id = 2;						// ALCT mez  pgm chain
	if (ichain==0x04) chain_id = 3;						// TMB  mez  FPGA + 4 PROMs, Virtex-2
	if (ichain==0x14) chain_id = 4;						// TMB  mez  FPGA + 2 PROMs, Virtex-6
	if (ichain==0x24) chain_id = 7;						// TMB  mez  FPGA + 2 PROMs, Spartan-6
	if (ichain==0x01) chain_id = 5;						// ALCT slow FPGA + 1 PROM,  Spartan-XL 
	if (ichain==0x13) chain_id = 6;						// ALCT mez  FPGA + 2 PROMS, Spartan-6
	if (ichain==0x00) chain_id = 8;						// ALCT slow User JTAG
	if (chain_id==-1) pause("vme_jtag_write_ir: chain_id not implemented");

	iframe = -1;										// Jtag frame number, 1st frame is 0 in c++
	for (preamble=0; preamble<4; ++preamble) {			// Set up for writing opcode
	iframe = iframe+1;									// Current frame number
	tms[iframe] = tms_pre_opcode[preamble];				// Take tap to ShfIR
	tdi[iframe] = tdi_pre_opcode[preamble];
	}

// Set up TMS to shift in the opcode bits for this chip, bypass code for others
	wr_opcode=opcode;									// Copy callers opcode
	ndevices=devices_in_chain[chain_id];				// Number of chips
	for (idevice=0; idevice<ndevices; ++idevice) {		// Loop over all chips
	ichip=ndevices-idevice-1;							// Chip order in chain is reversed
	nbitsop=bits_per_opcode[chain_id][ichip];			// # bits in its opcode
	for (ibit=0; ibit<nbitsop; ++ibit) {				// loop opcode bits
	if(ichip==chip_id)									// This is the chip we want
	  bit=(wr_opcode/(1 << ibit)) & 0x1;				// Extract bit
	  else												// All other chips
	  bit=1;											// Bypass

	 iframe=iframe+1;									// Current frame number
	 if(iframe > mxbits) pause("vme_write_ir: too many frames");
	 tms[iframe]=0;										// Tap stays in ShfIR
	 tdi[iframe]=bit;									// Instruction bit
	}													// Close for ibit
	}													// Close for idevice

// Set up TMS to take TAP from Ex1IR to RTI
	tms[iframe]=1;										// Last Opcode,Tap goes to Ex1IR
	for (postamble=0; postamble<2; ++postamble) {		// Take Tap to RTI
	iframe=iframe+1;									// Current frame number
	if (iframe > mxbits) pause("vme_write_ir: too many frames");
	tms[iframe]=tms_post_opcode[postamble];
	tdi[iframe]=tdi_post_opcode[postamble];
	}

// Write JTAG OpCodes, end up with tap in RTI
	step_mode=0	;										// 1=single step, 0=run
	nframes=iframe+1;									// Total frames to send
	vme_jtag_io_byte(adr,ichain,nframes,tms,tdi,tdo,step_mode);

// Check that TDO shifted out instruction register strings
	if ((ichain != 0x8) && (ichain != 0xC)) goto exit_write_ir;	// Skip if not TMB		

	iframe=3;											// Start at first opcode bit
	for (ichip=0; ichip<ndevices; ++ichip)				// Loop over all chips
	{
	nbitsop=bits_per_opcode[chain_id][ichip];			// # bits in its opcode
	 for (ibit=0; ibit<nbitsop; ++ibit) 				// loop opcode bits
	 {
	 iframe=iframe+1;									// Current opcode bit
	 if (iframe > mxbits) pause("vme_write_ir: too many frames");
	 iopbit=tdo[iframe];								// opcode bit read back
	 if ((iopbit!=prom_ir[ibit])&&(prom_ir[ibit]!= 2)) goto err_write_ir;
	 }													// Close for ibit
	}													// Close for ichip
	goto exit_write_ir;									// Opcode readback is ok

// Did not read back proper opcode sequence
// It should be 000xy001 000xy001: x=ISP status, y=Security
err_write_ir:

	for (i=12; i<=5; --i) {
	printf("vme_jtag_write_ir read bad ir: "); for (ibit=7;ibit>0;--ibit) printf("%1i",(tdo[i]>>ibit)&1);
	printf("\n");
	}

// Done
exit_write_ir:

	return;
}
//-------------------------------------------------------------------------------------------------------------------------------
//	Write TDI bits for the currently selected JTAG Data Register
//	for the specified chip.
//	Reads back TDO data.
//	
//	chip_id		=	device number to write
//	wr_data()	=	array to send containing nbits packed 1 bit per byte
//	rd_data()	=	read back data, 1 byte per bit
//	nbits		=	total number of bits to write
//
//	All chips except current chip are set to bypass mode.
//
//	It is presumed that the TAP controllers are presently in the RTI state
//	And that the desired opcode has already been written to the TAP.
//
//	JTAG instructions and data are loaded for
//	PROM1 first and PROM0 last:
//
//	JTAG				Load	TMB		RAT
//	Position Chip ID	Order	Name	Name
//	tdi>0		0		1		PROM0	FPGA
//	tdo<1		1		0		PROM1	PROM
//
//	05/08/2000	Changed tms_post_opcode to 2 bits instead of 3
//	10/26/2001	Adapted for TMB2001A Xilinx User PROMs
//	06/02/2004	Modified for multiple chains to support RAT and TMB
//	06/03/2004	Modified for TMB2005a mezzanine chain
//	10/09/2008	Convert to c++
//------------------------------------------------------------------------------
	void vme_jtag_write_dr(unsigned long &adr, int &ichain, int &chip_id, char wr_data[], char rd_data[], int &nbits)
{
	const int	mxbitstream = 1000;						// Max # bits in a jtag r/w cycle
	char		tms[mxbitstream];
	char		tdi[mxbitstream];
	char		tdo[mxbitstream];

	char		tms_pre_read[3]={1,0,0};				// RTI to ShfDR
	char		tdi_pre_read[3]={0,0,0};

	char		tms_post_read[2]={1,0};					// Ex1DR to RTI
	char		tdi_post_read[2]={0,0};

	int			reg_len;
	int			chain_id;
	int			offset;
	int			iframe;
	int			nframes;
	int			idevice;
	int			ichip;
	int			ibit;
	int			preamble;
	int			postamble;
	int			ndevices;
	int			step_mode;
	int			i;
//------------------------------------------------------------------------------
// Set up TMS to take TAP from RTI to Shift DR
	chain_id=-1;
	if (ichain==0x08) chain_id = 0;						// TMB user prom chain
	if (ichain==0x0C) chain_id = 0;						// TMB fpga chain
	if (ichain==0x0D) chain_id = 1;						// RAT chain
	if (ichain==0x02) chain_id = 2;						// ALCT mez  cfg chain
	if (ichain==0x03) chain_id = 2;						// ALCT mez  pgm chain
	if (ichain==0x04) chain_id = 3;						// TMB  mez  FPGA + 4 PROMs, Virtex-2
	if (ichain==0x14) chain_id = 4;						// TMB  mez  FPGA + 2 PROMs, Virtex-6
	if (ichain==0x24) chain_id = 7;						// TMB  mez  FPGA + 2 PROMs, Spartan-6
	if (ichain==0x01) chain_id = 5;						// ALCT slow FPGA + 1 PROM,  Spartan-XL 
	if (ichain==0x13) chain_id = 6;						// ALCT mez  FPGA + 2 PROMS, Spartan-6
	if (ichain==0x00) chain_id = 8;						// ALCT slow User JTAG
	if (chain_id==-1) pause("vme_jtag_write_ir: chain_id not implemented");

	iframe=-1;											// Jtag frame number, 1st frame is 0 in c++
	for (preamble=0; preamble<3; ++preamble) {			// Set up for writing opcode
	iframe=iframe+1;									// Current frame number
	tms[iframe]=tms_pre_read[preamble];					// Take tap to ShfDR
	tdi[iframe]=tdi_pre_read[preamble];
	}

// Set up TMS to shift in the data bits for this chip, bypass code for others
	reg_len=nbits;										// Register length
	ndevices=devices_in_chain[chain_id];				// Number of chips

	for (idevice=0; idevice<ndevices; ++idevice) {		// Loop over all chips
	ichip=ndevices-idevice-1;							// Chip order in chain is reversed
	 if (ichip == chip_id) {							// This is the chip we want
	  offset=iframe;									// Pointer to start of data
	  for (i=0; i<reg_len; ++i) {						// Clear out tdi
	  iframe=iframe+1;									// Add its register length
	  if (iframe > mxbitstream) pause("vme_write_dr: too many frames");
	  tms[iframe]=0;									// Stay in ShfDR
	  tdi[iframe]=wr_data[i] & 0x1;						// Data going out
	  }													// Close if ichip
	 }													// Close for i
	  else {											// All other chips
	  iframe=iframe+1;									// Bypass register is 1 frame
	  if (iframe > mxbitstream) pause("vme_write_dr: too many frames");
	  tms[iframe]=0;									// Stay in ShfDR
	  tdi[iframe]=0;									// No going out to bypass regs
	  }													// Close else(ichip
	}													// Close for idevice

	tms[iframe]=1;										// Last Opcode,Tap goes to Ex1DR
	for (postamble=0; postamble<2; ++postamble) {		// Take Tap to RTI
	iframe=iframe+1;									// Current frame number
	tms[iframe]=tms_post_read[postamble];
	tdi[iframe]=tdi_post_read[postamble];
	}

// Write JTAG Register Data, end up with tap in RTI
	step_mode=0;										//1=single step, 0=run
	nframes=iframe+1;									//Total frames to send
	vme_jtag_io_byte(adr,ichain,nframes,tms,tdi,tdo,step_mode);

// Copy relevant section of tdo to callers read array
	for (ibit=0; ibit<nbits; ++ibit) {
	iframe=ibit+offset+1;
	if (iframe > mxbitstream) pause("vme_write_dr: too many frames");
	rd_data[ibit]=tdo[iframe];
	if (ichain == 2) rd_data[ibit]=tdo[iframe-1];		// ALCT crap fix, idiots
	}

// Done
	return;
}
//------------------------------------------------------------------------------
// Return true if jtag programming cable is detected
//------------------------------------------------------------------------------
	bool vme_jtag_cable_detect (unsigned long &base_adr)
{
// Prototypes
	long int			vme_read (unsigned long &adr, unsigned short &rd_data);

// Local
	const unsigned long	vme_usr_jtag_adr = 0x000010;
	unsigned long int	adr;
	unsigned short		rd_data;
	long int			status;
	int					tdo_usr;

// Read fpga jtag register
	adr     = base_adr+vme_usr_jtag_adr;
	status  = vme_read(adr,rd_data);
	tdo_usr = (rd_data >> 15) & 0x1;

	if (tdo_usr==0) return true;	// cable forced tdo==0
	else            return false;	// cable absent
}
//------------------------------------------------------------------------------
// End of vme_jtag_io_ops
//------------------------------------------------------------------------------
