//------------------------------------------------------------------------------
// vme_io_wxp: Controls Bit3 for A24D16 read/write
///------------------------------------------------------------------------------
//	11/21/01 Initial
//	04/29/05 Modified for WinXP BT984 driver, added lines to define BT973 and opsys, mod swapping
//	03/10/06 Add block write
//	03/23/06 Add error print suppression
//	10/06/08 Change vme calls from pointers to references
//	11/05/08 Replace extern "C" with extern for cpp caller
//	01/25/10 Add block read
//	01/25/10 Add A24D16 mode
//	01/05/11 Add vme_sysreset
//------------------------------------------------------------------------------
// BIT3 definitions
//------------------------------------------------------------------------------
	#define BT973
	#define BT_WINNT

//------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------
	#include    <stdio.h>
	#include    "btapi.h"

//------------------------------------------------------------------------------
// Globals
//------------------------------------------------------------------------------
	bt_desc_t	btd;
	int			bt_open_success = 0;
	int			print_suppress  = 0;

//------------------------------------------------------------------------------
// vme_open:  opens Bit3 for A24D16
//------------------------------------------------------------------------------
	extern long int vme_open()
{
// Local
	bt_dev_t	type = BT_DEV_A24;
	int			unit = 0;
	char		devname[BT_MAX_DEV_NAME];
	bt_error_t	status;
	bt_swap_t	swapping = BT_SWAP_DEFAULT;

// Open device and logical unit
	bt_gen_name(unit,type,devname,BT_MAX_DEV_NAME);
	status = bt_open(&btd,devname, BT_RD | BT_WR);

	if (status != BT_SUCCESS)
	{	bt_perror(btd, status,"vme_open: bt_open could not open the device");
		return EXIT_FAILURE; }

// Clear any errors on interface
	status = bt_clrerr(btd);

	if (status != BT_SUCCESS)
	{	bt_perror(btd, status,"vme_open: bt_clrerr could not clear errors");
		(void) bt_close(btd);
		return EXIT_FAILURE; }

// Turn off byte swapping for bt984 winxp driver, its on by default, and wasn't on in 983 driver
	status= bt_set_info(btd,BT_INFO_SWAP,BT_SWAP_NONE);

	if (status != BT_SUCCESS)
	{	bt_perror(btd, status,"vme_open: bt_set_info could not set swap mode");
		return EXIT_FAILURE; }

// Set A24D16
	status= bt_set_info(btd,BT_INFO_DATAWIDTH,BT_WIDTH_D16);

	if (status != BT_SUCCESS)
	{	bt_perror(btd, status,"vme_open: bt_set_info could not set A24D16 mode");
		return EXIT_FAILURE; }

// Set global open-success flag for vme_close, otherwise, bt_close will violate
	bt_open_success = 1;
	return EXIT_SUCCESS;

}	// end vme_open

//------------------------------------------------------------------------------
// vme_write:  writes A24D16 data
//--------------------------------------------------------------------
	extern long int vme_write(unsigned long &adr, unsigned short &wr_data)
{
// Local
	size_t		xfer_length = 0x02;
	bt_error_t	status;
	size_t		xfer_done;

// Write data to the logical device
	status = bt_write(btd,(void *) &wr_data,adr,xfer_length,&xfer_done);
	if (status != BT_SUCCESS && print_suppress == 0)
	{	fprintf(stderr,"vme_write: bt_write error adr=%6.6X wr_data=%4.4X\n",adr,wr_data);
		return EXIT_FAILURE; }

	return EXIT_SUCCESS;
}

//------------------------------------------------------------------------------
// vme_bwrite:  writes A24D16 block data
//------------------------------------------------------------------------------
	extern long int vme_bwrite(unsigned long &adr, unsigned short wr_data[], long &nwords)
{
// Local
	size_t		xfer_length = nwords*2;	// xfer_length is number of bytes, 2 if D16 4 if D32
	bt_error_t	status;
	size_t		xfer_done;

// Write data to the logical device
	status = bt_write(btd,(void *) wr_data,adr,xfer_length,&xfer_done);

	if (status != BT_SUCCESS && print_suppress == 0)
	{	fprintf(stderr,"vme_bwrite: bt_write error \n");
		return EXIT_FAILURE; }
	
	return EXIT_SUCCESS;
}

//------------------------------------------------------------------------------
// vme_read:  reads A24D16 data
//------------------------------------------------------------------------------
	extern long int vme_read(unsigned long &adr, unsigned short &rd_data)
{
// Local
	size_t		xfer_length = 0x02;
	bt_error_t	status;
	size_t		xfer_done;

// Read from VME
   status = bt_read(btd,(void *) &rd_data,adr,xfer_length,&xfer_done);

	if ((status != BT_SUCCESS || xfer_done != xfer_length) && print_suppress == 0)
	{	fprintf(stderr,"vme_read:  bt_read  error adr=%6.6X rd_data=%4.4X\n",adr,rd_data);
		return EXIT_FAILURE; } 

	return EXIT_SUCCESS;
}

//------------------------------------------------------------------------------
// vme_bread:  reads A24D16 block data
//------------------------------------------------------------------------------
	extern long int vme_bread(unsigned long &adr, unsigned short rd_data[], long &nwords)
{
// Local
	size_t		xfer_length = nwords*2;	// xfer_length is number of bytes, 2 if D16 4 if D32
	bt_error_t	status;
	size_t		xfer_done;

// Read from VME
   status = bt_read(btd,(void *) rd_data,adr,xfer_length,&xfer_done);

	if ((status != BT_SUCCESS || xfer_done != xfer_length) && print_suppress == 0)
	{	fprintf(stderr,"vme_read:  bt_bread  error adr=%6.6X rd_data=%4.4X\n",adr,rd_data);
		return EXIT_FAILURE; } 
	
	return EXIT_SUCCESS;
}
//------------------------------------------------------------------------------
// vme_sysreset:  assert VME bus sysreset
//------------------------------------------------------------------------------
	extern long int vme_sysreset()
{
// Local
	bt_error_t      status;

// Fire sysreset
	status = bt_reset(btd);

	if (status != BT_SUCCESS && print_suppress == 0)
	{	fprintf(stderr,"vme_sysreset: bt_reset error \n");
		return EXIT_FAILURE; }
	
	return EXIT_SUCCESS;
}
//------------------------------------------------------------------------------
// vme_close:  closes Bit3 controller
//------------------------------------------------------------------------------
	extern long int vme_close()
{
// Local
	bt_error_t      status;

// Do the close
	if (bt_open_success == 1)
    status = bt_close(btd);

    if (status != BT_SUCCESS)
	{	bt_perror(btd, status, "vme_close: could not close the device");
		return EXIT_FAILURE; }

    return EXIT_SUCCESS;
}

//------------------------------------------------------------------------------
// vme_errs:	Change bus timeout error print
//				print_mode=0 prints bt errors
//				print_mode=1 suppresses error printing
//------------------------------------------------------------------------------
	extern long int vme_errs(const int &print_mode)
{
	print_suppress = print_mode;

    return EXIT_SUCCESS;
}
//------------------------------------------------------------------------------
// End vme_io_wxp.c
//------------------------------------------------------------------------------
