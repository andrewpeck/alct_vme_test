//------------------------------------------------------------------------------
//	Reads 3D3444 delay from VME register
//
//	06/04/09 Initial
//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	long int	vme_read	(unsigned long &adr, unsigned short &rd_data);

//------------------------------------------------------------------------------------------
// Function ddd_rd(base_adr,ddd_chip,ddd_channel) returns ddd_delay
//---------------------------------------------------------------------------------------------------------
	int ddd_rd(unsigned long &base_adr, const int &ddd_chip, const int &ddd_channel)
{
// VME address
	const unsigned long	vme_ddd0_adr =0x000016;

// Local
	long			status;
	unsigned long	adr;
	unsigned short	rd_data;
	int				ddd_delay;

// Read indicated delay register and extract 4-bit delay value
	adr	      = base_adr+vme_ddd0_adr+(2*ddd_chip);
	status    = vme_read(adr,rd_data);
	ddd_delay = (rd_data >> (ddd_channel*4)) & 0xF;

	return ddd_delay;
}
//------------------------------------------------------------------------------------------
// End ddd_rd
//------------------------------------------------------------------------------------------
