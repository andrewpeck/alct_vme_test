//------------------------------------------------------------------------------
//	ALCT384 Tests Using TMB+RAT
//------------------------------------------------------------------------------
//	04/12/2012	Initial
//	04/23/2012	Add single cable test
//	05/02/2012	Add ALCT digital serial number readout
//	05/07/2012	Add ADB hit list and CRC error readout
//	05/08/2012	Add single cable test patterns
//	06/20/2012	Add slow control FPGA walking 1
//	06/25/2012	Add fpga bit toggle
//------------------------------------------------------------------------------
//	Headers
//------------------------------------------------------------------------------
	#define _CRT_SECURE_NO_WARNINGS 1

	#include <stdio.h>
	#include <time.h>
	#include <windows.h>
	#include <math.h>
	#include <conio.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <iostream>
	#include <string>
	using namespace std;

//------------------------------------------------------------------------------
//	Common
//------------------------------------------------------------------------------
	FILE			*log_file;
	FILE			*unit;

	const int		mxframe	= 8192;		// Max raw hits frame number, scope adds 512*160/16=5120 frames
	const int		mxtbins	= 32;		// Highest time bin, 0 implies 32
	const int		mxly	= 6;		// # CSC Layers
	const int		mxds	= 8;		// # DiStrips per CFEB
	const int		mxdsabs	= 40;		// # DiStrips per CSC
	const int		mxcfeb	= 5;		// # CFEBs
	const int		mxbitstream=1000;	// Max # bits in a jtag cycle

// Common/decode_readout_common/
	int				scp_tbins;
	int				scp_playback;
	int				fifo_tbins_mini;
	int				first_event;
	int				itriad[mxtbins][mxdsabs][mxly];
	int				clct0_vme;
	int				clct1_vme;
	int				clctc_vme;
	int				mpc0_frame0_vme;
	int				mpc0_frame1_vme;
	int				mpc1_frame0_vme;
	int				mpc1_frame1_vme;
	int				nonzero_triads;
	int				adjcfeb_dist;
	int				mpc_me1a_block;

	int				expect_zero_alct;
	int				expect_zero_clct;
	int				expect_one_alct;
	int				expect_one_clct;
	int				expect_two_alct;
	int				expect_two_clct;
	int				expect_dupe_alct;
	int				expect_dupe_clct;
	int				vme_bx0_emu_en;
	bool			first_scn;

// Common/adc_common/
	double			adc_voltage[14];
	double			v5p0;
	double			v3p3;
	double			v1p5core;
	double			vcore_expect;
	double			vcore_noload;
	double			v1p5tt;
	double			v1p0;
	double			v1p0_expect;
	double			a5p0;
	double			a3p3;
	double			a1p5core;
	double			a1p5tt;
	double			a3p3rat;
	double			a1p8rat;
	double			v3p3rat;
	double			v1p8rat;
	double			vref2;
	double			vzero;
	double			vref;

// Common/adc_common_mez/
	double			adc_voltage_mez[14];
	double			v3p3_mez;
	double			v2p5_mez;
	double			vcore_mez;
	double			v1p8_mez;
	double			v1p2_mez;
	double			tfpga_mez;
	double			tsink_mez;
	double			vch07_mez;
	double			vch08_mez;
	double			vch09_mez;
	double			vch10_mez;
	double			vref2_mez;
	double			vzero_mez;
	double			vref_mez;

// Common/portsf_common/
	const int		setport_mxnwords = 4096;
	unsigned long	xilinx_boot_adr;
	unsigned short	xilinx_boot_data;
	int				setport_calls;
	int				setport_writes;
	int				setport_reads;
	int				setport_writes_expected;
	int				setport_nwords;
	int				setport_peak_nwords;
	unsigned short	setport_buffer[setport_mxnwords];
	int				xsvf_verbosity;
	bool			wlog;
	int				numwrites;
	int				numreads;

// Common/jtag_common/
	FILE			*jtag_file;
	bool			jtaglogmode;

//	Common/TMB_VME_addresses
	unsigned long		base_adr;
	const unsigned long	tmb_global_slot			= 26;
	const unsigned long	tmb_brcst_slot			= 27;
	const unsigned long	tmb_boot_adr			= 0x070000;
	//------------------------------------------------------------------------------
	const unsigned long	vme_idreg_adr			= 0x000000;	// For tmb2005 and bdtest_v5
	const unsigned long	vme_status_adr			= 0x000008;
	const unsigned long	vme_adr0_adr			= 0x00000A;
	const unsigned long	vme_adr1_adr			= 0x00000C;
	const unsigned long	vme_loopbk_adr			= 0x00000E;

	const unsigned long	vme_usr_jtag_adr		= 0x000010;
	const unsigned long	vme_prom_adr			= 0x000012;

	const unsigned long	vme_dddsm_adr			= 0x000014;
	const unsigned long	vme_ddd0_adr			= 0x000016;
	const unsigned long	vme_ddd1_adr			= 0x000018;
	const unsigned long	vme_ddd2_adr			= 0x00001A;
	const unsigned long	vme_dddoe_adr			= 0x00001C;
	const unsigned long	vme_ratctrl_adr			= 0x00001E;

	const unsigned long	vme_step_adr			= 0x000020;
	const unsigned long	vme_led_adr				= 0x000022;
	const unsigned long	vme_adc_adr				= 0x000024;
	const unsigned long	vme_dsn_adr				= 0x000026;
	//------------------------------------------------------------------------------
	const unsigned long	mod_cfg_adr				= 0x000028;	// For tmb2005 normal firmware
	const unsigned long	ccb_cfg_adr				= 0x00002A;
	const unsigned long	ccb_trig_adr			= 0x00002C;
	const unsigned long	ccb_stat0_adr			= 0x00002E;
	const unsigned long	alct_cfg_adr			= 0x000030;
	const unsigned long	alct_inj_adr			= 0x000032;
	const unsigned long	alct0_inj_adr			= 0x000034;
	const unsigned long	alct1_inj_adr			= 0x000036;
	const unsigned long	alct_stat_adr			= 0x000038;
	const unsigned long	alct_alct0_adr			= 0x00003A;
	const unsigned long	alct_alct1_adr			= 0x00003C;
	const unsigned long	alct_fifo_adr			= 0x00003E;
	const unsigned long	dmb_mon_adr				= 0x000040;
	const unsigned long	cfeb_inj_adr			= 0x000042;
	const unsigned long	cfeb_inj_adr_adr		= 0x000044;
	const unsigned long	cfeb_inj_wdata_adr		= 0x000046;
	const unsigned long	cfeb_inj_rdata_adr		= 0x000048;
	const unsigned long	hcm001_adr				= 0x00004A;
	const unsigned long	hcm023_adr				= 0x00004C;
	const unsigned long	hcm045_adr				= 0x00004E;
	const unsigned long	hcm101_adr				= 0x000050;
	const unsigned long	hcm123_adr				= 0x000052;
	const unsigned long	hcm145_adr				= 0x000054;
	const unsigned long	hcm201_adr				= 0x000056;
	const unsigned long	hcm223_adr				= 0x000058;
	const unsigned long	hcm245_adr				= 0x00005A;
	const unsigned long	hcm301_adr				= 0x00005C;
	const unsigned long	hcm323_adr				= 0x00005E;
	const unsigned long	hcm345_adr				= 0x000060;
	const unsigned long	hcm401_adr				= 0x000062;
	const unsigned long	hcm423_adr				= 0x000064;
	const unsigned long	hcm445_adr				= 0x000066;
	const unsigned long	seq_trig_en_adr			= 0x000068;
	const unsigned long	seq_trig_dly0_adr		= 0x00006A;
	const unsigned long	seq_trig_dly1_adr		= 0x00006C;
	const unsigned long	seq_id_adr				= 0x00006E;
	const unsigned long	seq_clct_adr			= 0x000070;
	const unsigned long	seq_fifo_adr			= 0x000072;
	const unsigned long	seq_l1a_adr				= 0x000074;
	const unsigned long	seq_offset0_adr			= 0x000076;
	const unsigned long	seq_clct0_adr			= 0x000078;
	const unsigned long	seq_clct1_adr			= 0x00007A;
	const unsigned long	seq_trig_src_adr		= 0x00007C;
	const unsigned long	dmb_ram_adr				= 0x00007E;
	const unsigned long	dmb_wdata_adr			= 0x000080;
	const unsigned long	dmb_wdcnt_adr			= 0x000082;
	const unsigned long	dmb_rdata_adr			= 0x000084;
	const unsigned long	tmb_trig_adr			= 0x000086;
	const unsigned long	mpc0_frame0_adr			= 0x000088;
	const unsigned long	mpc0_frame1_adr			= 0x00008A;
	const unsigned long	mpc1_frame0_adr			= 0x00008C;
	const unsigned long	mpc1_frame1_adr			= 0x00008E;
	const unsigned long	mpc_inj_adr				= 0x000090;
	const unsigned long	mpc_ram_adr				= 0x000092;
	const unsigned long	mpc_ram_wdata_adr		= 0x000094;
	const unsigned long	mpc_ram_rdata_adr		= 0x000096;
	unsigned long		scp_ctrl_adr			= 0x000098;
	unsigned long		scp_rdata_adr			= 0x00009A;
	const unsigned long	ccb_cmd_adr				= 0x00009C;

	const unsigned long	buf_stat0_adr			= 0x00009E;
	const unsigned long	buf_stat1_adr			= 0x0000A0;
	const unsigned long	buf_stat2_adr			= 0x0000A2;
	const unsigned long	buf_stat3_adr			= 0x0000A4;
	const unsigned long	buf_stat4_adr			= 0x0000A6;
	const unsigned long	alctfifo1_adr			= 0x0000A8;
	const unsigned long	alctfifo2_adr			= 0x0000AA;
	const unsigned long	seqmod_adr				= 0x0000AC;
	const unsigned long	seqsm_adr				= 0x0000AE;
	const unsigned long	seq_clctm_adr			= 0x0000B0;
	const unsigned long	tmbtim_adr				= 0x0000B2;
	const unsigned long	lhc_cycle_adr			= 0x0000B4;
	const unsigned long	rpc_cfg_adr				= 0x0000B6;
	const unsigned long	rpc_rdata_adr			= 0x0000B8;
	const unsigned long	rpc_raw_delay_adr		= 0x0000BA;
	const unsigned long	rpc_inj_adr				= 0x0000BC;
	const unsigned long	rpc_inj_adr_adr			= 0x0000BE;
	const unsigned long	rpc_inj_wdata_adr		= 0x0000C0;
	const unsigned long	rpc_inj_rdata_adr		= 0x0000C2;
	const unsigned long	rpc_tbins_adr			= 0x0000C4;
	const unsigned long	rpc_rpc0_hcm_adr		= 0x0000C6;
	const unsigned long	rpc_rpc1_hcm_adr		= 0x0000C8;
	const unsigned long	bx0_delay_adr			= 0x0000CA;
	const unsigned long	non_trig_adr			= 0x0000CC;
	const unsigned long	scp_trig_adr			= 0x0000CE;
	const unsigned long	cnt_ctrl_adr			= 0x0000D0;
	const unsigned long	cnt_rdata_adr			= 0x0000D2;

	const unsigned long	jtagsm0_adr				= 0x0000D4;
	const unsigned long	jtagsm1_adr				= 0x0000D6;
	const unsigned long	jtagsm2_adr				= 0x0000D8;

	const unsigned long	vmesm0_adr				= 0x0000DA;
	const unsigned long	vmesm1_adr				= 0x0000DC;
	const unsigned long	vmesm2_adr				= 0x0000DE;
	const unsigned long	vmesm3_adr				= 0x0000E0;
	const unsigned long	vmesm4_adr				= 0x0000E2;

	const unsigned long	dddrsm_adr				= 0x0000E4;
	const unsigned long	dddr0_adr				= 0x0000E6;

	const unsigned long	uptimer_adr				= 0x0000E8;
	const unsigned long	bdstatus_adr			= 0x0000EA;

	const unsigned long	bxn_clct_adr			= 0x0000EC;
	const unsigned long	bxn_alct_adr			= 0x0000EE;

	const unsigned long	layer_trig_adr			= 0x0000F0;
	const unsigned long	ise_version_adr			= 0x0000F2;

	const unsigned long	temp0_adr				= 0x0000F4;
	const unsigned long	temp1_adr				= 0x0000F6;
	const unsigned long	temp2_adr				= 0x0000F8;

	const unsigned long	parity_adr				= 0x0000FA;
	const unsigned long	ccb_stat1_adr			= 0x0000FC;
	const unsigned long	bxn_l1a_adr				= 0x0000FE;
	const unsigned long	l1a_lookback_adr		= 0x000100;
	const unsigned long	seq_debug_adr			= 0x000102;

	const unsigned long	alct_sync_ctrl_adr		= 0x000104;	// ALCT sync mode control
	const unsigned long	alct_sync_txdata_1st	= 0x000106;	// ALCT sync mode transmit data, 1st in time
	const unsigned long	alct_sync_txdata_2nd	= 0x000108;	// ALCT sync mode transmit data, 2nd in time

	const unsigned long	seq_offset1_adr			= 0x00010A;
	const unsigned long	miniscope_adr			= 0x00010C;

	const unsigned long	phaser0_adr				= 0x00010E;
	const unsigned long	phaser1_adr				= 0x000110;
	const unsigned long	phaser2_adr				= 0x000112;
	const unsigned long	phaser3_adr				= 0x000114;
	const unsigned long	phaser4_adr				= 0x000116;
	const unsigned long	phaser5_adr				= 0x000118;
	const unsigned long	phaser6_adr				= 0x00011A;

	const unsigned long	delay0_int_adr			= 0x00011C;
	const unsigned long	delay1_int_adr			= 0x00011E;

	const unsigned long	sync_err_ctrl_adr		= 0x000120;	// Synchronization Error Control

	const unsigned long	cfeb_badbits_ctrl_adr	= 0x000122;	// CFEB  Bad Bit Control/Status
	const unsigned long	cfeb_badbits_timer_adr	= 0x000124;	// CFEB  Bad Bit Check Interval

	const unsigned long	cfeb0_badbits_ly01_adr	= 0x000126;	// CFEB0 Bad Bit Array
	const unsigned long	cfeb0_badbits_ly23_adr	= 0x000128;	// CFEB0 Bad Bit Array
	const unsigned long	cfeb0_badbits_ly45_adr	= 0x00012A;	// CFEB0 Bad Bit Array

	const unsigned long	cfeb1_badbits_ly01_adr	= 0x00012C;	// CFEB1 Bad Bit Array
	const unsigned long	cfeb1_badbits_ly23_adr	= 0x00012E;	// CFEB1 Bad Bit Array
	const unsigned long	cfeb1_badbits_ly45_adr	= 0x000130;	// CFEB1 Bad Bit Array

	const unsigned long	cfeb2_badbits_ly01_adr	= 0x000132;	// CFEB2 Bad Bit Array
	const unsigned long	cfeb2_badbits_ly23_adr	= 0x000134;	// CFEB2 Bad Bit Array
	const unsigned long	cfeb2_badbits_ly45_adr	= 0x000136;	// CFEB2 Bad Bit Array

	const unsigned long	cfeb3_badbits_ly01_adr	= 0x000138;	// CFEB3 Bad Bit Array
	const unsigned long	cfeb3_badbits_ly23_adr	= 0x00013A;	// CFEB3 Bad Bit Array
	const unsigned long	cfeb3_badbits_ly45_adr	= 0x00013C;	// CFEB3 Bad Bit Array

	const unsigned long	cfeb4_badbits_ly01_adr	= 0x00013E;	// CFEB4 Bad Bit Array
	const unsigned long	cfeb4_badbits_ly23_adr	= 0x000140;	// CFEB4 Bad Bit Array
	const unsigned long	cfeb4_badbits_ly45_adr	= 0x000142;	// CFEB4 Bad Bit Array

	const unsigned long	last_vme_adr			= 0x00011E;	// Last valid address instantiated
	//------------------------------------------------------------------------------
	const unsigned long	vme_gpio_adr			= 0x000028;	// For bdtestv3
	const unsigned long	vme_cfg_adr				= 0x00002A;

	const unsigned long	cfeb0a_adr				= 0x00002C;	// Repeats 2C-48 for CFEBs1-4
	const unsigned long	cfeb0b_adr				= 0x00002E;
	const unsigned long	cfeb0c_adr				= 0x000030;
	const unsigned long	cfeb_offset_adr			= 0x000006;

	const unsigned long	alct_rxa_adr			= 0x00004A;
	const unsigned long	alct_rxb_adr			= 0x00004C;
	const unsigned long	alct_rxc_adr			= 0x00004E;
	const unsigned long	alct_rxd_adr			= 0x000050;

	const unsigned long	dmb_rxa_adr				= 0x000052;
	const unsigned long	dmb_rxb_adr				= 0x000054;
	const unsigned long	dmb_rxc_adr				= 0x000056;
	const unsigned long	dmb_rxd_adr				= 0x000058;

	const unsigned long	mpc_rxa_adr				= 0x00005A;
	const unsigned long	mpc_rxb_adr				= 0x00005C;

	const unsigned long	rpc_rxa_adr				= 0x00005E;
	const unsigned long	rpc_rxb_adr				= 0x000060;
	const unsigned long	rpc_rxc_adr				= 0x000062;
	const unsigned long	rpc_rxd_adr				= 0x000064;
	const unsigned long	rpc_rxe_adr				= 0x000066;
	const unsigned long	rpc_rxf_adr				= 0x000068;

	const unsigned long	ccb_rxa_adr				= 0x00006A;
	const unsigned long	ccb_rxb_adr				= 0x00006C;
	const unsigned long	ccb_rxc_adr				= 0x00006E;
	const unsigned long	ccb_rxd_adr				= 0x000070;

	const unsigned long	alct_txa_adr			= 0x000072;
	const unsigned long	alct_txb_adr			= 0x000074;

	const unsigned long	rpc_txa_adr				= 0x000076;
	const unsigned long	rpc_txb_adr				= 0x000078;

	const unsigned long	dmb_txa_adr				= 0x00007A;
	const unsigned long	dmb_txb_adr				= 0x00007C;
	const unsigned long	dmb_txc_adr				= 0x00007E;
	const unsigned long	dmb_txd_adr				= 0x000080;

	const unsigned long	mpc_txa_adr				= 0x000082;
	const unsigned long	mpc_txb_adr				= 0x000084;

	const unsigned long	ccb_txa_adr				= 0x000086;
	const unsigned long	ccb_txb_adr				= 0x000088;

	const unsigned long	heater_adr				= 0x00008A;	// Last bdtestv3 address instantiated

//------------------------------------------------------------------------------
//	Prototypes
//------------------------------------------------------------------------------
	#define			logical(L)		((L)?'T':'F')
	#define			yesno(L)		((L)?'y':'n')
	void			pause			(string s);
	void			stop			(string s);
	void			sleep			(clock_t msec);
	bool			pass_fail		(string prompt);

	void			idcode_decode	(long &idcode, string &sdevice_type, string &sdevice_name, string &sdevice_version, string &sdevice_size);
	void			dsn_rd			(unsigned long &vme_dsn_adr, const int &itype, int dsn[]);
	void			dsn_rd_alct		(unsigned long &adr, int &ichain, int &chip_id, int &opcode_rd, int &opcode_wr, int &reg_len, const int &itype, int dsn[]);
	void			dow_crc			(int in[7], int &crc);
	void			adc_read		(unsigned long &base_adr);
	void			adc_read_alct	(unsigned long &adr, int &ichain, int &chip_id, int &opcode_rd, int &opcode_wr, int &reg_len);
	void			aok				(string msg_string);
	void			aokf			(string msg_string, const int itest, const int status);	
	void			ck				(string data_string, int data_read, int data_expect);
	int				cks				(string data_string, int data_read, int data_expect);
	void			tok				(string msg_string, double fdata_read, double fdata_expect, double tolerance, int status);
	int				count0s			(char tdo[], int &nbits);
	int				count1s			(char tdo[], int &nbits);

	void			inquire			(string prompt, const int &minv, const int &maxv, const int &radix, int &now);
	void			inquir2			(string prompt, const int &minv, const int &maxv, const int &radix, int &num, int &now);
	void			inquirl			(string prompt, const int &minv, const int &maxv, const int &radix, long int &now);
	void			inquirb			(string prompt, bool &now);

	long int		vme_open		();
	long int		vme_read		(unsigned long &adr, unsigned short &rd_data);
	long int		vme_write		(unsigned long &adr, unsigned short &wr_data);
	long int		vme_sysreset	();
	long int		vme_close		();
	long int		vme_errs		(const int &print_mode);

	void			i4_to_tdi		(long int &i4, char  tdi[], const int &nbits, const int &spi);
	void			tdi_to_i4		(char  tdi[], long int &i4, const int &nbits, const int &spi);
	void			bit_to_array	(const int &idata, int iarray[], const int &n);

	int				xsvfExecute		();
	void			setPort			(short int p, short int val);
	unsigned char	readTDOBit		();

	void			vme_jtag_anystate_to_rti(unsigned long &adr, int &ichain);
	void			vme_jtag_write_ir		(unsigned long &adr, int &ichain, int &chip_id, int &opcode);
	void			vme_jtag_write_dr		(unsigned long &adr, int &ichain, int &chip_id, char wr_data[], char rd_data[], int &nbits);
	bool			vme_jtag_cable_detect	(unsigned long &base_adr);

//------------------------------------------------------------------------------
// File scope declarations
//------------------------------------------------------------------------------
// JTAG stream
	char			tdi[mxbitstream]={0};
	char			tdo[mxbitstream]={0};

// VME calls
	long			status;
//	unsigned long	base_adr;
	unsigned long	boot_adr;
	unsigned long	adr;
	unsigned short	rd_data;
	unsigned short	wr_data;

//------------------------------------------------------------------------------
// Event counters
//------------------------------------------------------------------------------
	const int		mxcounter = 79;
	int				cnt_lsb;
	int				cnt_msb;
	int				cnt_full;
	int				cnt[mxcounter];
	string			scnt[mxcounter];

//------------------------------------------------------------------------------
// Local
//------------------------------------------------------------------------------
	int				islot;
	int				islot_old;
	int				newslot;
	int				geo_adr;

// ID reg
	int				id_rev;
	int				id_rev_day;
	int				id_rev_month;
	int				id_rev_year;
	int				id_rev_fpga;

	int				id_slot;
	int				id_ver;
	int				id_type;
	int				id_month;
	int				id_day;
	int				id_year;

	int				id_reg[4];
	int				id_reg_save[4];

// Firmware
	unsigned short	tmb_firmware_type;
	unsigned short	tmb_firmware_series;
	unsigned short	tmb_firmware_normal		  = 0xC;
	unsigned short	tmb_firmware_debug		  = 0xD;
	unsigned short	tmb_firmware_series_etype = 0xE;

	unsigned short	ise_version;
	unsigned short	ise_sub;
	unsigned short	ise_sp;

	char			timestr[9];
	char			datestr[9];

	string			tmb_type;
	string			tmb_fpga_series;
	string			tmb_firmware_name;
	string			scomputer_name;

// Menu
	char			line[80];
	int				ifunc;
	int				i,j,k,n;

// Peek/poke
	char			rwe;
	unsigned long	newadr;
	unsigned long	newdata;

	int				nbang;
	int				ibang;
	bool			bang_read  = false;
	bool			bang_write = false;

// Debug
	bool			debug_loop;
	bool			debug_step;
	bool			debug_beep;

// ALCT single cable test
	int				itest;
	int				itest_first;
	int				itest_last;
	int				ipass;
	int				npasses;
	int				ipattern;
	int				npatterns;

	int				adb_wr_ch;
	long			adb_rd_ch;
	long			adb_rd_data;
	long			scsi_rd_data;
	int				scsi_wr_data;

	long			alct_fpga;
	long			alct_fmonthday;
	long			alct_fyear;
	long			alct_todd;
	long			alct_teven;
	long			alct_crc_err;
	long			adb_hit;
	long			adb_hit_expect;

	string			salct_fpga;
	string			sok;

	bool			iprint;
	bool			ifail;
	bool			adb_auto;
	bool			adb_passed[24];

	const int		alct_ntests=60;
	int				alct_npassed[alct_ntests+1];
	int				alct_nfailed[alct_ntests+1];
	int				alct_nskipped[alct_ntests+1];

// Slow control
	long			sc_begin;
	long			sc_end;
	int				ipin;
	int				ones;
	int				zeros;
	char			ipin_init[208+1];	// FPGA pins 1-208
	char			ipin_state[208+1];
	int				ipin_driven[208+1];
	int				ipin_shorted[208+1];
	int				ipin_skip[208+1];
	string			ipin_name[208+1];
	int				nshorted;
	int				iexpect;

// Trigger tests
	int				dmb_thresh_pretrig;
	int				hit_thresh_pretrig;
	int				hit_thresh_postdrift;
	int				pid_thresh_pretrig;
	int				pid_thresh_postdrift;
	int				lyr_thresh_pretrig;

	int				triad_persist;
	int				drift_delay;
	int				clct_sep;
	int				active_feb_src;
	int				active_feb_list;

	bool			layer_mode;
	int				layer_trig_en;

	int				fifo_mode;
	int				fifo_tbins;
	int				fifo_pretrig;

	int				fifo_tbins_rpc;
	int				fifo_pretrig_rpc;
	int				rpc_decouple;

	int				alct_bx0_en;
	int				alct_delay;
	int				clct_width;

	int				tmb_allow_clct;
	int				tmb_allow_alct;
	int				tmb_allow_match;

	int				tmb_allow_alct_ro;
	int				tmb_allow_clct_ro;
	int				tmb_allow_match_ro;

	int				inj_delay_rat;
	int				rpc_tbins_test;
	int				rpc_exists;

	bool			rrhd;
	bool			cprr;
	bool			cprr_ignore;
	bool			rat_injector_sync=false;
	bool			rat_injector_enable=true;
	bool			rpcs_in_rdout;
	bool			pause_on_fail;
	int				vme_bx0_emu_en_default=1;

	int				nclcts_inject=1;
	int				nalcts_inject=1;
	int				mcl;

	const int		mxclcts=8;
	int				clct_pat_inject[mxclcts];
	int				clct_pid_inject[mxclcts];
	int				clct_key_inject[mxclcts];
	int				clct_hit_inject[mxclcts];
	bool			clct_blanked[mxclcts];
	int				clct_hit_inject_clip[mxclcts];
	bool			loop_keys[mxclcts];
	bool			loop_pids[mxclcts];
	int				iclct;

	int				triad_1st_tbin[6]={0,0,0,0,0,0};
	int				l1a_delay=119;	// hits 0th l1a window bx
	int				rat_sync_mode;
	int				rat_injector_delay;

	int				irpc;
	int				ibxn;

	int				rpc_pad;
	int				rpc_inj_bxn;
	int				rpc_inj_image[256][2];
	int				rpc_inj_wen;
	int				rpc_inj_ren;
	int				rpc_inj_rwadr;
	int				rpc_inj_data;

// Scope
	bool			rdscope;
	int				scp_arm;
	int				scp_readout;
	int				scp_raw_decode;
	int				scp_silent;
	int				scp_raw_data[512*160/16];
	int				scp_auto;
	int				scp_nowrite;

// Misc
	int				boot_decode[16];
	unsigned short	boot_data;
	int				boot;

// Pattern cells
	int				icell;
	int				ihit;
	int				ihitp;

	int				nhits;
	int				layer;

	int				ikey;
	int				ikeylp;
	int				ikey_min;
	int				ikey_max;
	int				ikey_sep;

	int				ipid;
	int				ipidlp;
	int				ipid_min;
	int				ipid_max;

	int				icfeb;
	int				icfebg;
	int				key;
	int				ihs[6][160];
	int				nstag;

	int				idistrip;
	int				idslocal;
	int				itbin;
	int				itbin0;
	int				ihstrip;
	int				iram;
	int				pat_ram[32][3][5]={0};
	int				wen;
	int				ren;
	int				wadr;
	int				febsel;
	int				icfeblp;
	int				ibit;
	int				wr_data_mem;
	int				rd_data_mem;

	string			marker="AOXOMOXOA";

//------------------------------------------------------------------------------
	int				alct_data;
	int				alct_tdo;
	int				alct_lsbs;
	int				alct_expect;
	int				alct_err;
	int				alct_id;
	int				alct_id_expect;
	int				ireg;
	int				alct_injector_delay=14;	// experimental

	string			rat_chip_type[2]={"FPGA","PROM"};
	long int		rat_user1[7];
	long int		rat_user2[1];
	int				rat_ctrl_data;

// RAT USER1 status register
	char			rs[224];
	long int		rs_begin;
	long int		rs_version;
	long int		rs_monthday;
	long int		rs_year;

	long int		rs_syncmode;
	long int		rs_posneg;
	long int		rs_loop;

	long int		rs_rpc_en;
	long int		rs_clk_active;

	long int		rs_locked_tmb;
	long int		rs_locked_rpc0;
	long int		rs_locked_rpc1;
	long int		rs_locklost_tmb;
	long int		rs_locklost_rpc0;
	long int		rs_locklost_rpc1;

	long int		rs_txok;
	long int		rs_rxok;

	long int		rs_ntcrit;
	long int		rs_rpc_free;

	long int		rs_dsn;
	long int		rs_dddoe_wr;
	long int		rs_ddd_wr;
	long int		rs_ddd_auto;
	long int		rs_ddd_start;
	long int		rs_ddd_busy;
	long int		rs_ddd_verify_ok;

	long int		rs_rpc0_parity_ok;
	long int		rs_rpc1_parity_ok;
	long int		rs_rpc0_cnt_perr;
	long int		rs_rpc1_cnt_perr;
	long int		rs_last_opcode;

	long int		rw_rpc_en;
	long int		rw_ddd_start;
	long int		rw_ddd_wr;
	long int		rw_dddoe_wr;
	long int		rw_perr_reset;
	long int		rw_parity_odd;
	long int		rw_perr_ignore;
	long int		rw_rpc_future;

	long int		rs_rpc0_pdata;
	long int		rs_rpc1_pdata;

	long int		rs_unused;
	long int		rs_end;

// ALCT jtag opcodes
	int				IDRead;
	int				HCMaskRead;
	int				HCMaskWrite;
	int				RdTrig;
	int				WrTrig;
	int				RdCfg;
	int				WrCfg;
	int				Wdly;
	int				Rdly;
	int				CollMaskRead;
	int				CollMaskWrite;
	int				ParamRegRead;
	int				ParamRegWrite;
	int				InputEnable;
	int				InputDisable;
	int				YRwrite;
	int				OSread;
	int				SNread;
	int				SNwrite0;
	int				SNwrite1;
	int				SNreset;
	int				Bypass;

	string			alct_chip_type[2]={"FPGA","PROM"};
	string			tmb_chip_type[5] ={"FPGA","PROM","PROM","PROM","PROM"};
	long int		alct_cfgreg[3];
	long int		alct_idreg[2];
	char			rsa[69];
	int				alct_sn[2];
	int				alct_dsn_crc;
	int				alct_dsn_mfg;
	int				alct_dsn;
	int				ilen;
	int				ival;
	int				ivalarray[69];

	int				alct0_rd;
	int				alct1_rd;
	int				alct0_prev;
	int				alct1_prev;

	long int		din;
	long int		crc;
	long int		tmb_crc_lsb;
	long int		tmb_crc_msb;
	long int		tmb_crc;
	bool			crc_match;

	long int		rsa_trig_mode;
	long int		rsa_ext_trig_en;
	long int		rsa_pretrig_halt;
	long int		rsa_inject;
	long int		rsa_inject_mode;
	long int		rsa_inject_mask;
	long int		rsa_nph_thresh;
	long int		rsa_nph_pattern;
	long int		rsa_drift_delay;
	long int		rsa_fifo_tbins;
	long int		rsa_fifo_pretrig;
	long int		rsa_fifo_mode;
	long int		rsa_fifo_lastlct;
	long int		rsa_l1a_delay;
	long int		rsa_l1a_window;
	long int		rsa_l1a_offset;
	long int		rsa_l1a_internal;
	long int		rsa_board_id;
	long int		rsa_bxn_offset;
	long int		rsa_ccb_enable;
	long int		rsa_alct_jtag_ds;
	long int		rsa_alct_tmode;
	long int		rsa_alct_amode;
	long int		rsa_alct_maskall;
	long int		rsa_trig_info_en;
	long int		rsa_sn_select;

	long int		rsa_chip_id;
	long int		rsa_version;
	long int		rsa_year;
	long int		rsa_day;
	long int		rsa_month;

	char			rsd[84];
	long int		alct_user1[3];
	long int		alct_user2[2];
	long int		rsd_begin;
	long int		rsd_version;
	long int		rsd_monthday;
	long int		rsd_year;
	long int		rsd_mc_done;
	long int		rsd_sc_done;
	long int		rsd_clock_lock;
	long int		rsd_clock_en;
	long int		rsd_cmd_align;
	long int		rsd_cmd_sync_mode;
	long int		rsd_sync_mode;
	long int		rsd_sync_rx_1st_ok;
	long int		rsd_sync_rx_2nd_ok;
	long int		rsd_alct_rx_1st;
	long int		rsd_alct_rx_2nd;
	long int		rsd_cmd_l1a_en;
	long int		rsd_cmd_trig_en;
	long int		rsd_tx_en0;
	long int		rsd_tx_en1;
	long int		rsd_cmd_dummy;
	long int		rsd_free0;
	long int		rsd_end;

	int				err_alct_fifo_clr;
	int				err_alct_lct0;
	int				err_alct_lct1;
	int				err_alct_fifo_busy;
	int				err_alct_fifo_ndone;
	int				err_alct_raw_nwords;
	int				err_firmware_crc;
	int				err_alct_crc;
	int				err_lct;
	int				err_sum;
	int				err_lct_cmp;
	int				alct0_keya;
	int				alct1_keya;
	int				alct0_keyb;
	int				alct1_keyb;
	int				ievent;
	int				itrig_src;

	int				rpc_data[38];
	int				rpc_data_1st[38];
	int				rpc_data_2nd[38];
	int				rpc_delay_default;
	int				bad_1st;
	int				bad_2nd;
	int				rpc_bad[16];
	int				nbad;
	double			pctbad;

	bool			rpc_err;
	int				loopbk;
	int				itx;
	int				ijtag_src;

	int				rpc_rx[4];
	int				rpc_bxn[4];
	int				rpc_word[4];

	int				rpc_rbxn[4];
	int				rpc_rdata[4];
	int				rpc0_rdata_expect[19];
	int				rpc1_rdata_expect[19];
	int				rpc2_rdata_expect[19];
	int				rpc3_rdata_expect[19];
	int				irat;

	bool			dmb_err;
	int				mpc_err;
	int				ccb_data[64];
	int				ccb_err;
	int				reg_err;
	int				vme_cfg;
	int				vme_data;
	int				irx;

	string			vstat_5p0v;
	string			vstat_3p3v;
	string			vstat_1p8v;
	string			vstat_1p5v;
	string			tcrit;
	string			ok[2]={"BAD","OK "};
	string			sidcode_good;
	string			rat_user1_string;

	long int		i4;
	int				adr_mode;
	int				loopstate;

	int				sel_boot_jtag;
	int				sel_boot_nojtag;
	int				sel_step_alct;
	int				sel_step_cfeb;
	int				sel_loopbk;

	unsigned long	adc_adr;
	int				smb_adr;
	int				smb_cmd;
	int				smb_data;
	int				smb_data_tmb;
	int				smb_data_rat;

	int				opcode;
	int				opcode_rd;
	int				opcode_wr;
	int				reg_len;
	int				chip_id;
	int				nchips;
	long int		idcode;
	long int		usercode;
	int				user_idcode[2];
	int				ichain;
	int				idcode_v;
	int				idcode_f;
	int				idcode_a;
	int				idcode_c;

	string			sdevice_type;
	string			sdevice_name;
	string			sdevice_version;
	string			sdevice_size;

	const int		vme_mx_adr=254;
	int				vme_readback[vme_mx_adr];
	char			bell=7;

	int				icrc;
	int				itype;
	int				ichip;
	int				dsn_tmb[8];
	int				dsn_mez[8];
	int				dsn[8];
	string			dsn_chip[3]     ={"TMB ","TMEZ","RAT "};
	string			dsn_chip_alct[2]={"ALCT","AMEZ"};
	string			icrc_ok;

	int				dmb_wdcnt;
	int				dmb_busy;
	int				dmb_rdata;
	int				dmb_rdata_lsb;
	int				dmb_rdata_msb;

	int				cfeb_base;
	int				cfeb_data;
	int				cfeb_err;
	int				cfeb_id;
	int				cfeb_id_expect;

	int				iver;
	char			cfver[2+1];
	string			sfver;

	int				idsn;
	char			cdsn[7+1];
	string			sdsn;

//	const int		mxframe=4096;		// Max raw hits frame number
	int				vf_data[mxframe];
	int				iframe;
	int				ilayer;

	int				adr_e0b;
	int				r_nheaders;
	int				r_ncfebs;
	int				r_fifo_tbins;
	int				nsamples;

	int				read_pat[mxtbins][mxly][mxdsabs];
	int				rdcid;
	int				rdtbin;
	int				hits8;
	int				hits1;
	int				ids;
	int				ids_abs;
	int				jcfeb;
	char			x[]="          ";
	bool			display_cfeb;

	int				cfeb_rxdata_1st[24];
	int				cfeb_rxdata_2nd[24];
	int				cfeb_rxdata_1st_remap[24];
	int				cfeb_rxdata_2nd_remap[24];
	int				cfeb_sync_rxdata_1st;
	int				cfeb_sync_rxdata_2nd;

	int				pat_expect;
	int				clct_bxn_expect;

	int				clct_key_expect[mxclcts];
	int				clct_pid_expect[mxclcts];
	int				clct_hit_expect[mxclcts];

	int				m0def;
	int				m1def;
	int				muon0[256];
	int				muon1[256];
	int				nfdef;
	int				nframes;
	int				mpc_ram_wrdata[4];

	int				mpc_accept0;
	int				mpc_accept1;
	int				mpc_reserved0;
	int				mpc_reserved1;
	int				mpc_delay;
	int				wr_marker;

	char			cid[1];
	char			cidc[8];
	char			cich[1];
	char			cvcore[5];
	string			sid;
	string			sidc;
	string			sich;
	string			svcore;

	int				iflocal;
	int				ifhsink;
	char			cflocal[3+1];
	char			cfhsink[3+1];
	string			sflocal;
	string			sfhsink;

	int				ccb_rx_bank0;
	int				ccb_rx_bank1;
	int				ccb_rx_bank2;
	int				ccb_rx_bank3;

	unsigned short	ccb_rxa;
	unsigned short	ccb_rxb;
	unsigned short	ccb_rxc;
	unsigned short	ccb_rxd;

	unsigned short	dmb_rxa;
	unsigned short	dmb_rxb;
	unsigned short	dmb_rxc;
	unsigned short	dmb_rxd;

	int				wr_pat;
	int				wr_pat_vlad;
	int				wr_pat_ck;

	unsigned short	mpc_rxa;
	unsigned short	mpc_rxb;
	int				mpc_rx_bank0;
	int				mpc_rx_bank1;

	unsigned short	rpc_rxa;
	unsigned short	rpc_rxb;
	unsigned short	rpc_rxc;
	unsigned short	rpc_rxd;
	unsigned short	rpc_rxe;
	unsigned short	rpc_rxf;

	int				rpc_rxh[38];
	int				rpc_clock;
	int				rd_ddd0;
	int				rpc_smbrx;
	int				rpc_rx_bank0;
	int				rpc_rx_bank1;
	int				rpc_rx3126;
	int				rpc_rxh0902;
	int				rpc_dsn;
	int				rpc_done;

	int				smb_clk;
	int				tck_rpc;
	int				tms_rpc;
	int				tdi_rpc;
	int				tdo_rpc;
	int				rpc_sync;
	int				rpc_posneg;
	int				rpc_loop_tm;
	int				sel_rpc_chain;

	int				jtag_alct;
	int				alct_tx_lo;
	int				adb_pulse_async;
	int				nhard_reset_alct;
	int				alct_tx_hi;
	int				alct_loop;
	int				alct_rxoe;
	int				alct_txoe;
	int				alct_clock_en;
	int				alct_clock;
	int				rpc_free_tx0;
	int				nhard_reset_rpc;

	int				ipass_full_auto;
	double			amptol;
	double			ttol;
	int				isource;

	int				clct_sm;
	int				read_sm;
	string			sclct_sm[6];
	string			sread_sm[22];

	int				hit_thresh_pretrig_temp;
	int				hit_thresh_postdrift_temp;

	int				fmm_state;
	string			sfmm_state[5]={"Startup","Resync ","Stop   ","WaitBXO","Run    "};

	bool			rdraw;
	int				nbxn0;
	int				ntrig;

	int				nperbank;
	int				ibank;
	int				alct_1st_bank[3];
	int				alct_2nd_bank[3];
	int				ifs;

	int				alct_raw_busy;
	int				alct_raw_done;
	int				alct_raw_nwords;
	int				alct_raw_data;

	int				alct0_raw_lsb;
	int				alct0_raw_msb;
	int				alct0_raw;

	int				alct1_raw_lsb;
	int				alct1_raw_msb;
	int				alct1_raw;

	string			dmb_chip[4]={"U28","U29","U30","U31"};
	string			rpc_chip[2]={"U42","U43"};

	int				prom_clk[2];
	int				prom_oe[2];
	int				prom_nce[2];
	int				iprom;
	int				jprom;
	int				prom_data;
	int				prom_src;
	int				prom_adr;

	int				wr_buf_ready;
	int				buf_stalled;
	int				buf_q_full;
	int				buf_q_empty;
	int				buf_q_ovf_err;
	int				buf_q_udf_err;
	int				buf_q_adr_err;
	int				buf_display;
	int				wr_buf_adr;
	int				buf_fence_dist;
	int				buf_fence_cnt;
	int				buf_fence_cnt_peak;
	int				buf_free_space;

	int				queue_full;
	int				queue_empty;
	int				queue_ovf;
	int				queue_udf;

	int				seqdeb_adr;
	int				seqdeb_rd_mux;
	unsigned long	deb_adr_diff;
	unsigned long	deb_wr_buf_adr;
	unsigned long	deb_buf_push_adr;
	unsigned long	deb_buf_pop_adr;
	unsigned long	deb_buf_push_data;
	unsigned long	deb_buf_pop_data;

	int				push_l1a_bxn_win;
	int				push_l1a_cnt_win;
	int				push_l1a_match_win;
	int				push_l1a_push_me;
	int				push_l1a_notmb;
	int				push_tmb_nol1a;
	int				push_wr_buf_avail;

	int				pop_l1a_bxn_win;
	int				pop_l1a_cnt_win;
	int				pop_l1a_match_win;
	int				pop_l1a_push_me;
	int				pop_l1a_notmb;
	int				pop_tmb_nol1a;
	int				pop_wr_buf_avail;

	int				crc_err;
	int				crc_err_old;

	int				rat_window_width;
	int				rat_window_open;
	int				rat_window_close;
	int				rat_window_center;
	int				rat_window_nbad[16];
	int				err_bit;
	int				rat_board_id;
	int				irtest;
	int				imode;

	const int		mxadcerr=32;
	int				adc_err[mxadcerr+1];

	char			cbid[4+1];
	string			sbid;
	int				lenv;
	char			tmb_logfolder[81];
	char			rat_logfolder[81];

	int				islot_dut;
	int				islot_ref;
	unsigned long	base_adr_ref;
	unsigned long	boot_adr_ref;
	unsigned long	base_adr_dut;
	unsigned long	boot_adr_dut;
	unsigned long	base_adr_global;
	unsigned long	boot_adr_global;
	unsigned long	base_adr_chk;
	unsigned long	boot_adr_chk;
	unsigned long	adr_ww1;

	bool			tmb_ref_exists;
	bool			tmb_ref_skip;
	int				tmb_board_id;
	int				tmb_firmware_type_ref;
	int				statid[10];

	const int		tmb_ntests=60;
	int				tmb_npassed[tmb_ntests+1];
	int				tmb_nfailed[tmb_ntests+1];
	int				tmb_nskipped[tmb_ntests+1];

	int				tmb_npass=0;
	int				tmb_nfail=0;
	int				tmb_nskip=0;

	const int		rat_ntests=19;
	int				rat_npassed[rat_ntests+1];
	int				rat_nfailed[rat_ntests+1];
	int				rat_nskipped[rat_ntests+1];

	int				rat_npass;
	int				rat_nfail;
	int				rat_nskip;

	double			diff;
	double			vtol;
	char			ckey;

	int				geo_adr_rd;
	int				geo_parity;
	int				parity;

	int				radix;
	char			csize[3+1];
	string			ssize;
	char			czsize[4+1];
	string			szsize;
	char			cprom[1+1];
	string			sprom;

	unsigned char	outbuf[16];
	char			colon=':';

	string			log_file_name;
	FILE*			img_file;

	char			cid_rev[4+1];
	string			sid_rev="byte";

	string			logfolder;
	string			jtag_file_name;

	int				rec_len;
	int				rec_type;
	int				adr_hi_byte;
	int				adr_lo_byte;
	int				segment;
	int				cksum;
	int				nwords;

	int				ibyte;
	int				filler;
	int				nwrite;
	int				iadr_previous;

	int				nerrors;
	int				nwrites;
	int				iadr;
	int				cmp_data;
	int				cmp_adr;

	int				jtag_data;
	int				jtag_tck;
	int				jtag_tms;
	int				jtag_tdi;
	int				jtag_sel;

	int				alct_begin_marker;
	int				alct_end_marker;
	int				alct_end_header;
	int				alct_unass;
	int				alct_type;
	int				alct_month;
	int				alct_day;
	int				alct_year;
	int				alct_version;
	int				imonth;
	int				iday;
	int				iyear;

	int				wdcnt;
	int				prom_wdcnt;
	int				prom_cksum;
	int				njtag_frames;

	string			sresult_cksum;
	string			sresult_wdcnt;
	bool			first_prom_pass;

	int				uptime;
	int				uptime_hr;
	int				uptime_min;
	int				uptime_sec;

	int				bd_status_ok;
	int				bd_vstat_5p0v;
	int				bd_vstat_3p3v;
	int				bd_vstat_1p8v;
	int				bd_vstat_1p5v;
	int				bd_t_crit;

	int				iadr_rd;
	int				rd_data_rd;
	int				nmiss;
	int				nadrs;
	int				nadrs_written;
	int				nadrs_read;
	int				blue_flash;

	int				alct_bx0_delay;
	int				clct_bx0_delay;

	bool			compare_prom_file;
	bool			fire_injector=false;
	bool			err_check=true;
	bool			skip_loopback_series=false;

	unsigned short	wr_fire_l1a;

	int				l1a_lookback;
	int				hdr_wr_continuous;
	int				lookback_triad_hits[2048]={0};

	int				first_nonzero_bx;
	int				last_nonzero_bx;
	int				first_bx;
	int				last_bx;
	int				max_triads;
	double			scale;
	int				tck;

 	int				xsvf_file_size;
	string			xsvf_file_name;
	string			xsvf_tmb_user_default="userproms.xsvf";
	struct			_stat buf;
	int				stat_result;

	clock_t			startClock;
	clock_t			endClock;
	unsigned char	ucTdoBit;
	short int		TMS=1;

//------------------------------------------------------------------------------
	int main()
{
//------------------------------------------------------------------------------
//	Debug print mode
//------------------------------------------------------------------------------
//	#define debug_print 1	// comment this line to turn off debug print

	#ifdef debug_print
	 #define dprintf fprintf
	#else
	 #define dprintf  //
	#endif

//------------------------------------------------------------------------------
//	Open log file
//------------------------------------------------------------------------------
// Log file
	log_file_name = "alct_test_log.txt";
	log_file      = fopen(log_file_name.c_str(),"w");
//	setbuf(log_file, NULL);	// stops buffering, but is 3x slower

	if (log_file  != NULL)  dprintf(stdout,"Opened      %s\n",log_file_name.c_str());
	if (log_file  == NULL) {fprintf(stdout,"Failed to open %s\n",log_file_name.c_str()); pause("WTF?");}

// Put date into log files
	_strtime(timestr);
	_strdate(datestr);

	fprintf(stdout,"\tStarted:       %s %s\n",datestr,timestr);
	fprintf(log_file,"Started:       %s %s\n",datestr,timestr);

//------------------------------------------------------------------------------
//	Open VME Interface 
//------------------------------------------------------------------------------
	bool vme_opened=false;

	status    = vme_open();					// Connect to Bit3 VME controller
	islot     = tmb_global_slot;			// TMB VME slot address
	base_adr  = islot << 19;				// VME base address  for this slot
	boot_adr  = base_adr | tmb_boot_adr;	// VME boot register for this slot
	vme_opened= (status==0);

//------------------------------------------------------------------------------
// Get computer name
//------------------------------------------------------------------------------
	const int		infoBuflen=32;
	char			infoBuf[infoBuflen];
	unsigned long	bufCharCount;

	bufCharCount = infoBuflen;								// Initial length
 	GetComputerName(infoBuf,&bufCharCount);					// Returns actual length
	if (bufCharCount>=infoBuflen) infoBuf[infoBuflen-1]=0;	// Manually null terminate
	scomputer_name = string(infoBuf);

//------------------------------------------------------------------------------
//	Get TMB firmware type code
//------------------------------------------------------------------------------
begin:
	adr    = base_adr;
	status = vme_read(adr,rd_data);
	tmb_firmware_type   = (rd_data >> 0) & 0xF;
	tmb_firmware_series = (rd_data >> 4) & 0xF;
	geo_adr             = (rd_data >> 8) & 0x1F;

	tmb_type = "TMB2XXX";				//any non-tmb2005 board
	if (tmb_firmware_series == tmb_firmware_series_etype) tmb_type= "TMB2005";

// Decode id_rev
	adr    = base_adr+6;
	status = vme_read(adr,rd_data);
	id_rev			= rd_data;
	id_rev_day		= (id_rev >>  0) & 0x001F;
	id_rev_month	= (id_rev >>  5) & 0x000F;
	id_rev_year		= (id_rev >>  9) & 0x000F;
	id_rev_fpga		= (id_rev >> 13) & 0x0007;
	id_rev_fpga		=  id_rev_fpga+2;

// Get TMB FPGA series
	if (id_rev_fpga==5) id_rev_fpga=6;

	tmb_fpga_series="Unknown";
	if (id_rev_fpga==3) tmb_fpga_series="XC6SLX150";
	if (id_rev_fpga==4) tmb_fpga_series="XC2V4000";
	if (id_rev_fpga==6) tmb_fpga_series="XC6VLX195T";

// Get TMB compiler version
	adr    = base_adr + ise_version_adr;	// Recent TMBs have ISE register
	status = vme_read(adr,rd_data);
	ise_version = (rd_data >> 8) & 0xFF;
	ise_sub     = (rd_data >> 4) & 0xF;
	ise_sp      = (rd_data >> 0) & 0xF;

	if (ise_version<8 && ise_version!=4){	// Old TMBs were all FND4.2sp3 and do not have an ISE register
	ise_version = 4;
	ise_sub     = 2;
	ise_sp      = 3;
	}

	if (tmb_firmware_type == tmb_firmware_debug) {	// Debug TMBs are all ISE8.2sp3 and do not have an ISE register
	ise_version = 8;
	ise_sub     = 2;
	ise_sp      = 3;
	}

// Determine TMB firmware type
	tmb_firmware_name = "TMB200x type unknown...beware";							// Unknown TMB type

	if      (id_rev_fpga==3||id_rev_fpga==4||id_rev_fpga==6) {
	if	    (tmb_firmware_type==tmb_firmware_debug ) tmb_firmware_name = "Debug Loopback";	// Debug TMB type
	else if (tmb_firmware_type==tmb_firmware_normal) tmb_firmware_name = "Normal        ";	// Normal TMB type
	}

// Log TMB type
	fprintf(log_file,"TMB  Firmware  %s\n",tmb_firmware_name.c_str());
	fprintf(log_file,"TMB  RevCode   %4.4X=%2.2i/%2.2i/%2.2i\n",id_rev,id_rev_month,id_rev_day,id_rev_year);
	fprintf(log_file,"TMB  FPGA      %s series %1i\n",tmb_fpga_series.c_str(),id_rev_fpga);
	fprintf(log_file,"TMB  ISE       %2.2X.%1X SP%1.1X\n",ise_version,ise_sub,ise_sp);
	fprintf(log_file,"C++  Compiled  %s\n\n",__TIMESTAMP__);

//------------------------------------------------------------------------------
//	Main Menu
//------------------------------------------------------------------------------
main_menu:
	printf("\tALCT Test Menu Host %s\n",scomputer_name.c_str());
	printf("\tTMB  Firmware  %s\n",tmb_firmware_name.c_str());
	printf("\tTMB  RevCode   %4.4X=%2.2i/%2.2i/%2.2i\n",id_rev,id_rev_month,id_rev_day,id_rev_year);
	printf("\tTMB  FPGA      %s series %1i\n",tmb_fpga_series.c_str(),id_rev_fpga);
	printf("\tTMB  ISE       %2.2X.%1X SP%1.1X\n",ise_version,ise_sub,ise_sp);
/*
	printf("\tALCT Firmware  %s\n",tmb_firmware_name.c_str());
	printf("\tALCT RevCode   %4.4X=%2.2i/%2.2i/%2.2i\n",id_rev,id_rev_month,id_rev_day,id_rev_year);
	printf("\tALCT FPGA      %s\n",tmb_fpga_series.c_str());
	printf("\tALCT ISE       %2.2X.%1X SP%1.1X\n",ise_version,ise_sub,ise_sp);
	printf("\tC++  Compiled  %s\n\n",__TIMESTAMP__);
*/
	printf("\t1:   TMB Slot    [%2.2i] adr=%6.6X\n",islot,base_adr);
	printf("\t2:   Read TMB ID Register\n");
	printf("\t3:   Read TMB Boot Register\n");
	printf("\t4:   Peek/Poke VME Address\n");
	printf("\t7:   Hard Reset TMB\n");
	printf("\t8:   Hard Reset ALCT\n");
	printf("\t10:  Walking 1 Tests\n");
	printf("\t11:  Read Spartan-6 ADC\n");
	printf("\t12:  Read FPGA and PROM IDcodes\n");
	printf("\t13:  Read Hardware Serial Numbers\n");
	printf("\t14:  Single Cable Test\n");
	printf("\t15:  Slow Control Walking 1 Test\n");
	printf("\t16:  Slow Control Toggle 1 bit\n");
	printf("\t<cr> Exit\n");
	printf("       > ");

	gets(line);
	if (line[0]==NULL) goto exit;
	sscanf(line,"%i",&ifunc);

	i=abs(ifunc);
	if (i== 1) {void L100();	L100();		goto begin;    }
	if (i== 2) {void L200();	L200();		goto main_menu;}
	if (i== 3) {void L300();	L300();		goto main_menu;}
	if (i== 4) {void L400();	L400();		goto main_menu;}
	if (i== 7) {void L700();	L700();		goto begin;    }
	if (i== 8) {void L800();	L800();		goto main_menu;}
	if (i==10) {void L1000();	L1000();	goto main_menu;}
	if (i==11) {void L1100();	L1100();	goto main_menu;}
	if (i==12) {void L1200();	L1200();	goto main_menu;}
	if (i==13) {void L1300();	L1300();	goto main_menu;}
	if (i==14) {void L1400();	L1400();	goto main_menu;}
	if (i==15) {void L1500();	L1500();	goto main_menu;}
	if (i==16) {void L1600();	L1600();	goto main_menu;}

	goto main_menu;

//------------------------------------------------------------------------------
// 	Exit main: Close VME Interface
//------------------------------------------------------------------------------
exit:
	if (vme_opened    ) status = vme_close();	// Close VME
	if (log_file!=NULL) fclose(log_file);		// Close log file

	printf("\tSic transit gloria mundi");		// Say goodbye
	gets(line);

	return 0;
}
//------------------------------------------------------------------------------
//	Change TMB Slot
//------------------------------------------------------------------------------
	void L100() {
L100:
	printf("\tOld slot=%2.2i New slot[0-31]=",islot);

	gets(line);
	if (line[0]==NULL) goto L100;
	sscanf(line,"%i",&newslot);
	if ((newslot>31) || (newslot<0)) goto L100;

	islot    = newslot;
	base_adr = (islot << 19);
	boot_adr = base_adr | tmb_boot_adr;

	return;
}
//------------------------------------------------------------------------------
//	Read TMB ID Register
//------------------------------------------------------------------------------
	void L200() {
//L200:
	
	if (ifunc < 0) goto L250;	// Bang mode
	for (i=0; i<=3; ++i) {
	adr    = base_adr + vme_idreg_adr + 2*i;
	status = vme_read(adr,rd_data);
	printf("\tadr=%6.6X read=%4.4X\n",adr,rd_data);
	id_reg[i]=rd_data;
	}

	id_slot = (id_reg[0] >> 8) & 0x00FF;
	id_ver  = (id_reg[0] >> 4) & 0x000F;
	id_type = (id_reg[0] >> 0) & 0x000F;
	id_month= (id_reg[1] >> 8) & 0x00FF;
	id_day  = (id_reg[1] >> 0) & 0x00FF;
	id_year =  id_reg[2];
	id_rev  =  id_reg[3];

// Decode id_rev
	id_rev_day		= (id_rev >>  0) & 0x001F;
	id_rev_month	= (id_rev >>  5) & 0x000F;
	id_rev_year		= (id_rev >>  9) & 0x000F;
	id_rev_fpga		= (id_rev >> 13) & 0x0007;
	id_rev_fpga		= id_rev_fpga+2;
	if (id_rev_fpga==5) id_rev_fpga=6;

	printf("\n");
	printf("\tid_slot=%2.2X=%2.2d\n",id_slot,id_slot);
	printf("\tid_rev =%1X\n",  id_ver );
	printf("\tid_type=%1X\n",  id_type);
	printf("\tid_date=%2.2X/%2.2X/%4.4X\n",id_month,id_day,id_year);
	printf("\tid_rev =%4.4X=%2.2i/%2.2i/%2.2i series %1i\n",id_rev,id_rev_month,id_rev_day,id_rev_year,id_rev_fpga);

	printf("\n\t<cr> to continue:");
	gets(line);

	return;

// Bang mode reads id reg and compares to previous reads
L250:
	nbang=10000;

	for (ibang=1; ibang<=nbang; ++ibang) {
	if (ibang%10000==0) printf("ibang=%i\n",ibang);

	for (i=0; i<=3; ++i) {
	adr    = base_adr + vme_idreg_adr + 2*i;
	status = vme_read(adr,rd_data);
	id_reg[i]=rd_data;

// Compare id reg to previous read
	if ((ibang != 1) && (id_reg[i] != id_reg_save[i])) {
	printf("id bang fail: ibang=%7i i=%1i read=%4.4X expect=%4.4X\n",ibang,i,id_reg[i],id_reg_save[i]);
	}	// close if ibang
	id_reg_save[i]=id_reg[i];
	}	// close i
	}	// close ibang
	goto L250;
	}

//------------------------------------------------------------------------------
//	Read TMB Boot Register
//------------------------------------------------------------------------------
	void L300() {
L300:

	status = vme_read(boot_adr,rd_data);	// Get current boot reg
	if (ifunc < 0) goto L300;				// Bang mode

	printf("\tBoot=%4.4X Adr=%6.6X\n",rd_data,boot_adr);

	for (i=0; i<=15; ++i) {
	boot_decode[i]=(rd_data >> i) & 0x1;
	}

	i=0;
	printf("\n");
	printf("\t[%2.2i]%2i  R/W jtag_vme1  (tdi) vme tdi\n",				i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W jtag_vme2  (tms) vme tms\n",				i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W jtag_vme3  (tck) vme tck\n",				i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W sel_vme0    00XX ALCT JTAG Chain\n",		i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W sel_vme1    01XX TMB Mezzanine FPGA\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W sel_vme2    10XX TMB User PROMs JTAG\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W sel_vme3    11XX TMB FPGA User JTAG\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W vme/usr_en  1=JTAG sourced by Boot\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W HardRstAlct 1=Hard reset to ALCT FPGA\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W HardRstTmb  1=Hard reset to TMB FPGA\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W /EnResetAlct0=Allow TMB reset ALCT\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W /FpgaVmeEn  1=Allow TMB to issue VME\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W /MezClockEn 0=Enable TMB mez clock\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R/W HardResetRpc1=Hard reset to RAT\n",		i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R   vme_ready   1=FPGA vme logic ready\n",	i,boot_decode[i]); i++;
	printf("\t[%2.2i]%2i  R   jtag_vme0   (tdo) vme tdo\n",				i,boot_decode[i]); i++;

	printf("\n\t<cr> to continue:");
	gets(line);

	return;
}
//------------------------------------------------------------------------------
//	Peek/Poke an arbitrary VME address
//------------------------------------------------------------------------------
	void L400() {
L400:

	bang_read =false;
	bang_write=false;

	printf("\tPeek/Poke(hex): read=r,adr | write=w,adr,wrdata | inc adr=<cr> | e=exit\n");
	adr=base_adr;

L420:
	rd_data=0xFFFF;		// clear out previous read, bit 3 doesn't update rd_data if no dtack
	status = vme_read(adr,rd_data);
	if (bang_read) goto L420;

	printf("\tadr=%6.6X data=%4.4X r/w/e,adr,wrdata=",adr,rd_data);
	gets(line);
	n=strlen(line);
	dprintf(stdout,"length=%i\n",n);

// <cr> = increment address
	if (line[0]==NULL) {
	adr=adr+2;
	goto L420;
	}

// Parse input string into adr and wr_data
	if (n < 1) goto L400;

	sscanf(line,"%c,%X,%X",&rwe,&newadr,&newdata);
	dprintf(stdout,"rwe=%c newadr=%6.6X newdata=%4.4X\n",rwe,newadr,newdata);

// e=exit
	if ((rwe=='e') || (rwe=='E')) return;
	if ((rwe=='R') || (rwe=='W')) printf("\tBang mode....\n");

//	Check adr is even and 24 bits or less and data is 16 bits or less
	if ((newadr >> 24) != 0) {
	printf("dumbass: address exceeds 24 bits\n");
	goto L420;
	}
	if ((newadr & 0x1) != 0) {
	printf("dumbass: address must be even\n");
	goto L420;
	}

// If address is for slot 0, assume it is relative to current base address
	if (newadr < 0x080000)newadr=newadr | base_adr;

/// r = read data at new address
	if (rwe=='R')bang_read=true;
	if ((rwe=='r') || (rwe=='R')) {	
	adr=newadr;
	goto L420;
	}

// w = write data at address
	if (rwe=='W')bang_write=true;
	if ((rwe=='w') || (rwe=='W')) {

	if ((newdata >> 16) != 0) {
	printf("dumbass: write data exceeds 16 bits\n");
	goto L420;
	}

	adr=newadr;
	wr_data=(unsigned short)newdata;
L480:	
	status = vme_write(adr,wr_data);
	if (bang_write) goto L480;
	goto L420;
	}
	goto L400;
}
//------------------------------------------------------------------------------
//	Hard Reset TMB
//------------------------------------------------------------------------------
	void L700() {
L700:

	status  = vme_read(boot_adr,rd_data);			// Get current boot reg
	wr_data = rd_data | 0x0200;						// Turn on  TMB hard reset
	status  = vme_write(boot_adr,wr_data);			// Assert   TMB hard reset

	wr_data = rd_data & ~0x0200;					// Turn off TMB hard reset
	status  = vme_write(boot_adr,wr_data);			// Restore boot reg
//	sleep(150);										// Wait for TMB to reload, Virtex2 takes 100ms
	sleep(450);										// Wait for TMB to reload, Virtex6 takes 400ms
	if (ifunc <0 ) goto L700;						// Bang mode
	return;											// Get latest firmware type
}
//------------------------------------------------------------------------------
//	Hard Reset ALCT
//------------------------------------------------------------------------------
	void L800() {
L800:

	status  = vme_read(boot_adr,rd_data);			// Get current boot reg
	wr_data = rd_data | 0x0100;						// Turn on  ALCT hard reset
	status  = vme_write(boot_adr,wr_data);			// Assert   ALCT hard reset

	wr_data = rd_data & ~0x0100;					// Turn off ACLT hard reset
	status  = vme_write(boot_adr,wr_data);			// Restore boot reg
	sleep(150);										// Wait for ALCT to reload
	if (ifunc < 0) goto L800;						// Bang mode
	return;
}
//------------------------------------------------------------------------------
//	Read DMB Raw Hits RAM from tmb2004a logic
//------------------------------------------------------------------------------
	void L900() {
L900:

	if (tmb_firmware_type != tmb_firmware_normal) {
	pause("TMB has wrong firmware type for this operation");
	return;
	}

// Get DMB RAM word count and busy bit
	adr    = dmb_wdcnt_adr+base_adr;
	status = vme_read(adr,rd_data);

	dmb_wdcnt = rd_data & 0x0FFF;
	dmb_busy  = (rd_data >> 14) & 0x0001;

	printf("\tword count = %4i\n",dmb_wdcnt);
	printf("\tbusy       = %4i\n",dmb_busy);
	printf("\tword count = %4i\n",dmb_wdcnt);
	printf("\tbusy       = %4i\n",dmb_busy);

	if (dmb_busy != 0) {
	pause ("Can not read RAM: dmb reports busy");
	goto L910;
	}

	if (dmb_wdcnt <= 0) {
	pause("Can not read RAM: dmb reports word count <=0");
	goto L910;
	}

// Write RAM read address to TMB
	for (i=0; i<=(dmb_wdcnt-1); ++i) { 
	adr     = dmb_ram_adr+base_adr;
	wr_data = i & 0xFFFF;
	status  = vme_write(adr,wr_data);

// Read RAM data from TMB
	adr    = dmb_rdata_adr+base_adr;
	status = vme_read(adr,rd_data);			// read lsbs
	dmb_rdata_lsb = rd_data;

	adr    = dmb_wdcnt_adr+base_adr;
	status = vme_read(adr,rd_data);			// read msbs
	dmb_rdata_msb = (rd_data >> 12) & 0x3;	// rdata msbs
	dmb_rdata     = dmb_rdata_lsb | (dmb_rdata_msb << 16);

	fprintf(log_file,"Adr=%4i Data=%5.5X\n",dmb_rdata);
	fprintf(stdout,  "Adr=%4i Data=%5.5X\n",dmb_rdata);
//	if (i%16 != 0) pause("next");
	}	// close for i

//	Clear RAM address, reset for next event
L910:
	adr     = dmb_ram_adr+base_adr;
	wr_data = 0x2000;	// reset RAM write address
	status  = vme_write(adr,wr_data);
	wr_data = 0x0000;	// unreset
	status  = vme_write(adr,wr_data);

// Bang mode
	if (ifunc < 0) goto L900;
	return;
}
//------------------------------------------------------------------------------
//	ALCT Transition Module Walking 1 Tests:
//------------------------------------------------------------------------------
	void L1000 () {
//L10600:
	printf("\tNot implemented yet\n");
	return;

	sel_boot_jtag = 0x0080;
	sel_step_alct = 0x1D10;

// Put ALCT into normal mode, and enable ALCT transition module (disable SCSI)
	wr_data = 0x000F;	// TMB2005
	adr		= vme_loopbk_adr + base_adr;
	status	= vme_write(adr,wr_data);

// Loop over 29 receive bits, only do full tx test with bit 0
	for (irx=0; irx<=28; ++irx)	// 10620
	{

// Loop over 23 tx bits
	j=23;
	if (irx==0) j=0;

	for (i=j; i<=23; ++i)
	{
	printf("\t\t ALCT Transition testing bit alct_tx%2.2i with alct_rx%2.2i%c\n",i,irx,bell);

L10610:
	alct_err=0;

	for (ipass=1; ipass<=5000; ++ipass) {
	for (itx  =0; itx  <=1;    ++itx  ) {

// Clear previous data on ALCT outputs
	wr_data = 0;
	status	= vme_write(adr=tmb_boot_adr+base_adr, wr_data=sel_boot_jtag);
	status	= vme_write(adr=alct_txa_adr+base_adr, wr_data=wr_data);
	status	= vme_write(adr=alct_txb_adr+base_adr, wr_data=wr_data);
	status	= vme_write(adr=vme_step_adr+base_adr, wr_data=sel_step_alct);

// Toggle tx bit and send to ALCT output registers
	if (i>=0 && i<=4)
	{
	 wr_data= (itx<<i) | sel_boot_jtag;
	 adr	= tmb_boot_adr + base_adr;
	 status	= vme_write(adr,wr_data);
	}
	else if (i>=5 && i<=17)
	{
	 wr_data= (itx << (i-5));
	 adr	= alct_txa_adr + base_adr;
	 status	= vme_write(adr,wr_data);
	}
	else if (i==18)	// inverted data
	{
	 wr_data= ((itx^1)<<8) | sel_boot_jtag;
	 adr	= tmb_boot_adr + base_adr;
	 status	= vme_write(adr,wr_data);
	}
	else if (i>=19 || i<=24)
	{
	 wr_data= (itx<<(i-19));
	 adr	= alct_txb_adr + base_adr;
	 status	= vme_write(adr,wr_data);
	}
	else
	{
	 stop("dumbass");
	}

// Read ALCT input registers
	alct_data=0;

	adr		= base_adr+tmb_boot_adr;
	status	= vme_read(adr,rd_data);
	alct_data = alct_data | ((rd_data>>15) & 0x1); // rx0 is TDO

	for (ireg=0; ireg<=3; ++ireg)
	{
	adr		  = base_adr+alct_rxa_adr+2*ireg;
	status	  = vme_read(adr,rd_data);
	rd_data   = rd_data & 0xFF;
	alct_data = alct_data | (rd_data << (ireg*8));
	}
	rd_data   = (alct_data>>irx) & 0x1;

// Check that transmitted bit is received OK, can't check if others are 0, because they float
	if (rd_data!=itx) alct_err=1;
	if (itx!=0 && itx!=1) pause("wtf!");
	printf("ALCT error i=%i3 itx=%i3 rd_data=%4.4X\n",i,itx,rd_data);

// Close loops
	}	// close for itx
	if (alct_err!=0) goto L10610;
	}	// close ipass
	}	// close i
	}	// close irx 10620

// ALCT passes
	printf("\t\tALCT Transition Module passed walking 1%c\n",bell);
	return;
}

//------------------------------------------------------------------------------
//	Read Spartan-6 ADC
//------------------------------------------------------------------------------
	void L1100() {
//L1100:
// Get current ADC values
	ichain    = 0x2;		// ALCT Mezzanine control jtag chain
	adr       = boot_adr;	// Boot register address
	chip_id   = 0;			// ALCT user path has 1 chip

	opcode_rd = 0x08;		// ALCT mez ADC read  opcode
	opcode_wr = 0x09;		// ALCT mez ADC write opcode
	reg_len   = 5;			// Data register length

	adc_read_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len);

// Display Spartan-6 mezzanine ADC
	fprintf(stdout,"\n");
	fprintf(stdout,"\tSpartan6 ADC\n");
	fprintf(stdout,"\t+3.3   Vcc      %6.3f V\n",v3p3_mez);
	fprintf(stdout,"\t+2.5   Vccaux   %6.3f V\n",v2p5_mez);
	fprintf(stdout,"\t+1.5|8 Vcore    %6.3f V\n",vcore_mez);
	fprintf(stdout,"\t+1.8   Vccprom  %6.3f V\n",v1p8_mez);
	fprintf(stdout,"\t+1.2   Vccint   %6.3f V\n",v1p2_mez);
	fprintf(stdout,"\t+Tfpga          %6.3f C  %5.1f F\n",tfpga_mez,(32.+tfpga_mez*9./5.));
	fprintf(stdout,"\t+Tsink          %6.3f C  %5.1f F\n",tsink_mez,(32.+tsink_mez*9./5.));
	fprintf(stdout,"\t+vref/2         %6.3f V\n",vref2_mez);
	fprintf(stdout,"\t+vzero          %6.3f V\n",vzero_mez);
	fprintf(stdout,"\t+vref           %6.3f V\n",vref_mez);

	printf("\n\t<cr> to continue:");
	gets(line);
	return;
}
//------------------------------------------------------------------------------
//	Read FPGA and PROM ID codes from TMB, RAT, and ALCT
//------------------------------------------------------------------------------
	void L1200() {
L1200:

//--
// TMB Mezzanine FPGA and PROM Chain
//--
	if      (id_rev_fpga==3) {ichain=0x24; nchips=3;}		// Spartan6 TMB Mezzanine pgm jtag chain
	else if (id_rev_fpga==6) {ichain=0x14; nchips=3;}		// Virtex6  TMB Mezzanine pgm jtag chain
	else                     {ichain=0x04; nchips=5;}		// Virtex2  TMB Mezzanine pgm jtag chain

	adr=boot_adr;
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI

// Read Virtex2 FPGA (6-bit opcode) and XC18V04 PROM IDcodes (8-bit opcode)
	for (chip_id=0; chip_id<nchips; ++chip_id) {
	if  (chip_id==0) opcode = 0x09;							// FPGA IDcode opcode
	if  (chip_id>=1) opcode = 0xFE;							// PROM IDcode opcode
	reg_len = 32;											// IDcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(tdo,idcode,32,0);								// Deserialize
	idcode_decode (idcode, sdevice_type, sdevice_name, sdevice_version, sdevice_size);

// Read FPGA/PROM USERCodes
	if  (chip_id==0) opcode = 0x08;							// FPGA USERcode opcode
	if  (chip_id>=1) opcode = 0xFD;							// PROM USERcode opcode
	reg_len = 32;											// USERcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],usercode,32,0);						// Deserialize

	fprintf(stdout,"\tTMB  Mez  Device=%1i IDcode=%8.8X %s Name=%s\tVer=%s\tSize=%s USERcode=%8.8X\n",
	chip_id, idcode, sdevice_type.c_str(), sdevice_name.c_str(), sdevice_version.c_str(), sdevice_size.c_str(),usercode);

// Close for chip_id
	}	
	fprintf(stdout,"\n");

//--
// TMB User PROM Chain
//--
	ichain = 0x0008;										// User PROM chain
	nchips = 2;
	adr    = boot_adr;
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI

// Read User PROM IDcodes XC18V256/XC18V512 (8-bit opcode)
	for (chip_id=0; chip_id<nchips; ++chip_id) {
	opcode  = 0xFE;											// IDcode opcode
	reg_len = 32;											// IDcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(tdo,idcode,32,0);								// Deserialize
	idcode_decode (idcode, sdevice_type, sdevice_name, sdevice_version, sdevice_size);

// Read PROM USERCodes
	opcode  = 0xFD;											// PROM USERcode opcode
	reg_len = 32;											// USERcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],usercode,32,0);						// Deserialize

	fprintf(stdout,"\tTMB  User Device=%1i IDcode=%8.8X %s Name=%s\tVer=%s\tSize=%s USERcode=%8.8X\n",
	chip_id, idcode, sdevice_type.c_str(), sdevice_name.c_str(), sdevice_version.c_str(), sdevice_size.c_str(),usercode);

// Close for chip_id
	}	
	fprintf(stdout,"\n");

//--
// RAT FPGA and PROM Chain
//--
	ichain = 0x000D;										// RAT jtag chain
	nchips = 2;
	adr    = boot_adr;
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI

// Read Spartan IIE FPGA (5 bit opcode) and PROM (8 bit opcode) IDcodes 
	for (chip_id=0; chip_id<nchips; ++chip_id) {
	if  (chip_id==0) opcode = 0x09;							// FPGA IDcode opcode
	if  (chip_id>=1) opcode = 0xFE;							// PROM IDcode opcode
	reg_len = 32;											// IDcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(tdo,idcode,32,0);								// Deserialize
	idcode_decode (idcode, sdevice_type, sdevice_name, sdevice_version, sdevice_size);

// Read FPGA/PROM USERCodes
	if  (chip_id==0) opcode = 0x08;							// FPGA USERcode opcode
	if  (chip_id>=1) opcode = 0xFD;							// PROM USERcode opcode
	reg_len = 32;											// USERcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],usercode,32,0);						// Deserialize

	fprintf(stdout,"\tRAT  Modu Device=%1i IDcode=%8.8X %s Name=%s\tVer=%s\tSize=%s USERcode=%8.8X\n",
	chip_id, idcode, sdevice_type.c_str(), sdevice_name.c_str(), sdevice_version.c_str(), sdevice_size.c_str(),usercode);

// Close for chip_id
	}	
	fprintf(stdout,"\n");

//--
// ALCT Mezzanine FPGA and PROM Chain
//--
//	ichain = 0x03;											// ALCT Virtex-E mezzanine jtag chain
//	nchips = 2;
	ichain = 0x13;											// ALCT Spartan-6 mezzanine jtag chain
	nchips = 3;
	adr    = boot_adr;
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI

// Read Spartan-6 FPGA (6-bit opcode) and XCF32P,XCF08P PROM IDcodes (16-bit opcode)
	for (chip_id=0; chip_id<nchips; ++chip_id) {
	if  (chip_id==0) opcode = 0x09;							// FPGA IDcode opcode
	if  (chip_id>=1) opcode = 0xFE;							// PROM IDcode opcode
	reg_len = 32;											// IDcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(tdo,idcode,32,0);								// Deserialize
	idcode_decode (idcode, sdevice_type, sdevice_name, sdevice_version, sdevice_size);

// Read FPGA/PROM USERCodes
	if  (chip_id==0) opcode = 0x08;							// FPGA USERcode opcode
	if  (chip_id>=1) opcode = 0xFD;							// PROM USERcode opcode
	reg_len = 32;											// USERcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],usercode,32,0);						// Deserialize

	fprintf(stdout,"\tALCT Mez  Device=%1i IDcode=%8.8X %s Name=%s\tVer=%s\tSize=%s USERcode=%8.8X\n",
	chip_id, idcode, sdevice_type.c_str(), sdevice_name.c_str(), sdevice_version.c_str(), sdevice_size.c_str(),usercode);

// Close for chip_id
	}	
	fprintf(stdout,"\n");

//--
// ALCT Slow Control FPGA and PROM Chain
//--
	ichain = 0x01;											// ALCT mezzanine jtag chain
	nchips = 2;
	adr    = boot_adr;
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI

// Read Spartan 40XL FPGA (3-bit opcode) and XC18V01 PROM IDcodes (8-bit opcode)
	for (chip_id=0; chip_id<nchips; ++chip_id) {
	if  (chip_id==0) opcode = 0x6;							// FPGA IDcode opcode
	if  (chip_id>=1) opcode = 0xFE;							// PROM IDcode opcode
	reg_len = 32;											// IDcode length

	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(tdo,idcode,32,0);								// Deserialize
	idcode_decode (idcode, sdevice_type, sdevice_name, sdevice_version, sdevice_size);
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(tdo,idcode,32,0);								// Deserialize
//	printf("\topcode=%2.2X idcode=%8.8X\n",opcode,idcode);

// Read FPGA/PROM USERCodes
	if  (chip_id==0) {usercode=0xDEADD00D; goto skipxl;}	// FPGA USERcode opcode, Spartan XL has no usercode, so reuse IDcode
	if  (chip_id>=1) opcode = 0xFD;							// PROM USERcode opcode
	reg_len = 32;											// USERcode length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],usercode,32,0);						// Deserialize

skipxl:
	fprintf(stdout,"\tALCT Slow Device=%1i IDcode=%8.8X %s Name=%s\tVer=%s\tSize=%s USERcode=%8.8X\n",
	chip_id, idcode, sdevice_type.c_str(), sdevice_name.c_str(), sdevice_version.c_str(), sdevice_size.c_str(),usercode);

// Close for chip_id
	}	
	fprintf(stdout,"\n");

// Bang mode 
	if (ifunc>0) {printf("\t<cr> to continue:"); gets(line);}
	if (ifunc<0) goto L1200;
	return;
}
//------------------------------------------------------------------------------
//	Read ALCT Hardware Serial Numbers
//------------------------------------------------------------------------------
void L1300() {
// L1300:
// Set posneg=0, enable rat dsn
	adr    = vme_ratctrl_adr+base_adr;
	status = vme_read(adr,rd_data);
	rat_ctrl_data = rd_data;

	wr_data = rd_data & 0xFFFD;		// [0]=sync_mode [1]=posneg [2]=loop_tmb [3]=free_tx0 [4]=dsn en
	wr_data = wr_data | 0x0010;		// enable dsn bit
	status  = vme_write(adr,wr_data);

// Enable RAT
	adr    = vme_loopbk_adr+base_adr;
	status = vme_read(adr,rd_data);
	loopstate = rd_data;

	wr_data = rd_data | 0x000C;		//  set front/rear=0 to enable RAT
	status  = vme_write(adr,wr_data);

// Loop over TMB+RAD dsn chips
	for (itype=0; itype<=2; ++itype) {
	adr = base_adr+vme_dsn_adr;
	dsn_rd(adr,itype,dsn);

// Check CRC
	dow_crc(dsn,icrc);
	icrc_ok="ERR!";

	if (icrc==dsn[7] && icrc!=0) icrc_ok = "OK  ";
	printf("\tDigital Serial for %s CRC=%2.2X DSN=%",dsn_chip[itype].c_str(),dsn[7]);
	for (i=5; i>0; i--) fprintf(stdout,"%2.2X",dsn[i]);
	printf(" MFG=%2.2X %s\n",dsn[0],icrc_ok.c_str());

	}	// close itype

// Put posneg back
	adr     = vme_ratctrl_adr+base_adr;
	wr_data = rat_ctrl_data;
	status  = vme_write(adr,wr_data);

// Put loop state back
	adr     = vme_loopbk_adr+base_adr;
	wr_data = loopstate;
	status  = vme_write(adr,wr_data);
					
// Read ALCT digital serial numbers
	ichain     = 0x2;								// ALCT Mezzanine control jtag chain
	adr        = boot_adr;							// Boot register address
	chip_id    = 0;	

	opcode_rd  = 0x06;								// ALCT opcode
	opcode_wr  = 0x07;								// ALCT opcode
	reg_len    = 10;								// Register length

	for (itype=0; itype<=1; ++itype)
	{
	dsn_rd_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len,itype,dsn);
	dow_crc(dsn,icrc);

	if (icrc==dsn[7] && icrc!=0) icrc_ok = "OK  ";
	else                     	 icrc_ok = "ERR!";

	printf("\tDigital Serial for %s CRC=%2.2X DSN=%",dsn_chip_alct[itype].c_str(),dsn[7]);
	for (i=5; i>0; i--) printf("%2.2X",dsn[i]);
	printf(" MFG=%2.2X %s\n",dsn[0],icrc_ok.c_str());
	}
	printf("\n");
}
//------------------------------------------------------------------------------
//	Single Cable test
//
//	Requires alct_sctest.v firmware in the ALCT mezzanine FPGA
//	IR width is 5 bits
//	DR width varies
//
//	Opcode	Direction	Width		Data
//	------	---------	-----	--------------------------
//	0x0		bypass		1		Bypass
//	0x01	read		16		FPGA type			0x600E
//	0x02	read		16		Monthday[15:0]		0x0423
//	0x03	read		16		Year[15:0]			0x2012
//	0x04	read		16		Todd[15:0]			0x5555
//	0x05	read		16		Teven[15:0]			0xAAAA
//
//	0x06	read		10		dsn[9:0]
//	0x07	write		10		dsn[9:0]
//
//	0x08	read		5		adc[4:0]
//	0x09	write		5		adc[4:0]
//
//	0x0A	read		24		adb_hit[23:0]
//	0x0B	read		1		crc_error[0:0]

//	0x15	read		9		adb_adr[8:0]		ADB  connector channel readback
//	0x16	write		9		adb_adr[8:0]		ADB  connector channel 0-27
//
//	0x17	read		16		scsi_rd_data[15:0]	SCSI data readback
//	0x18	write		16		scsi_wr_data[15:0]	SCSI data to write, looped back to ADB via cable 
//
//	0x19	read		16		adb_rd_data[15:0]	ADB  data read back via Delay ASIC and multiplexers
//------------------------------------------------------------------------------
void L1400()
{
//L1400:
// Initialize channel and passes
	adb_wr_ch    = 0;										// Initial ADB channel
	adb_auto     = false;
	npasses      = 1;
	npatterns    = 1;
	scsi_wr_data = 0x1234;

	ichain  = 0x2;											// ALCT Mezzanine control jtag chain
	adr     = boot_adr;										// Boot register address
	chip_id = 0;											// ALCT user path has 1 chip

// Create fat 0 for writing to data registers
	for (i=0; i<mxbitstream; ++i) tdi[i]=0;
	
// Read ALCT FPGA type
	opcode  = 0x01;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	//fprintf(stdout,"\ttdo="); for (i=0; i<reg_len; ++i) fprintf(stdout,"%1i",tdo[i]); fprintf(stdout,"\n");

	tdi_to_i4(&tdo[0],alct_fpga,reg_len,0);

	if      (alct_fpga==0x600E) salct_fpga = "Virtex";
	else if (alct_fpga==0x1506) salct_fpga = "Spartan";
	else                        salct_fpga = "Unknown";

	printf("\tALCT FPGA type         %4.4X %s\n",alct_fpga,salct_fpga.c_str());

// Read ALCT firmware monthday
	opcode  = 0x02;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],alct_fmonthday,reg_len,0);
	printf("\tALCT firmware monthday %4.4X\n",alct_fmonthday);

// Read ALCT firmware year
	opcode  = 0x03;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],alct_fyear,reg_len,0);
	printf("\tALCT firmware year     %4.4X\n",alct_fyear);

// Read ALCT Todd
	opcode  = 0x04;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],alct_todd,reg_len,0);
	printf("\tALCT Todd              %4.4X\n",alct_todd);

// Read ALCT Teven
	opcode  = 0x05;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],alct_teven,reg_len,0);
	printf("\tALCT Teven             %4.4X\n",alct_teven);

// Read ALCT SEU induced CRC error
	opcode  = 0x0B;											// ALCT opcode
	reg_len = 1;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],alct_crc_err,reg_len,0);
	printf("\tALCT SEU CRC Error     %4.1X\n",alct_crc_err);
	printf("\n");

// Read digital serial numbers
	opcode_rd  = 0x06;										// ALCT opcode
	opcode_wr  = 0x07;										// ALCT opcode
	reg_len    = 10;										// Register length

	for (itype=0; itype<=1; ++itype)
	{
	dsn_rd_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len,itype,dsn);
	dow_crc(dsn,icrc);

	if (icrc==dsn[7] && icrc!=0) icrc_ok = "OK  ";
	else                     	 icrc_ok = "ERR!";

	printf("\tDigital Serial for %s CRC=%2.2X DSN=%",dsn_chip_alct[itype].c_str(),dsn[7]);
	for (i=5; i>0; i--) printf("%2.2X",dsn[i]);
	printf(" MFG=%2.2X %s\n",dsn[0],icrc_ok.c_str());
	}

// Read Spartan-6 ADC
	if (alct_fpga!=0x600E)	// Skip ADC if FPGA is known to be Virtex-E
	{
	opcode_rd = 0x08;		// ALCT mez ADC read  opcode
	opcode_wr = 0x09;		// ALCT mez ADC write opcode
	reg_len   = 5;			// Data register length

	adc_read_alct(adr,ichain,chip_id,opcode_rd,opcode_wr,reg_len);

	fprintf(stdout,"\n");
	fprintf(stdout,"\tSpartan6 ADC\n");
	fprintf(stdout,"\t+3.3   Vcc      %6.3f V\n",v3p3_mez);
	fprintf(stdout,"\t+2.5   Vccaux   %6.3f V\n",v2p5_mez);
	fprintf(stdout,"\t+1.5-8 Vcore    %6.3f V\n",vcore_mez);
	fprintf(stdout,"\t+1.8   Vccprom  %6.3f V\n",v1p8_mez);
	fprintf(stdout,"\t+1.2   Vccint   %6.3f V\n",v1p2_mez);
	fprintf(stdout,"\t+Tfpga          %6.3f C  %5.1f F\n",tfpga_mez,(32.+tfpga_mez*9./5.));
	fprintf(stdout,"\t+Tsink          %6.3f C  %5.1f F\n",tsink_mez,(32.+tsink_mez*9./5.));
	fprintf(stdout,"\t+vref/2         %6.3f V\n",vref2_mez);
	fprintf(stdout,"\t+vzero          %6.3f V\n",vzero_mez);
	fprintf(stdout,"\t+vref           %6.3f V\n",vref_mez);
	}

L1400_menu:
// Display menu
	printf("\n");
	printf("\tALCT Single Cable Test on ADB ch=%2i npasses=%i\n",adb_wr_ch,npasses);
	printf("\t0:   Set ADB channel and passes\n");
	printf("\t1:   Custom data\n");
	printf("\t2:   Running '1'\n");
	printf("\t3:   Running '0'\n");
	printf("\t4:   Filling by '1's\n");
	printf("\t5:   Filling by '0's\n");
	printf("\t6:   Shifting '5's and 'A's\n");
	printf("\t7:   Random data\n");
	printf("\t8:   All tests\n");
	printf("\t9:   All tests with automatic ADB increment\n");
	printf("\t<cr> Return to main menu\n");
	printf("      > ");

	gets(line);
	if (line[0]==NULL) return;
	sscanf(line,"%i",&itest);

	if (itest<0 || itest>9) goto L1400_menu;

	if (itest==9) {adb_auto=true; npasses=10;}
	else          {adb_auto=false;}

// Set ADB channel and npasses
	if (itest==0)
	{
	printf ("\tSet ADB channel and npasses:\n\n");
	inquire("\tADB Channel 0-23  %2i", 0,  23, 10, adb_wr_ch);
	inquire("\tTest Passes 0=inf %i" , 0,1000, 10, npasses);
	goto L1400_menu;
	}

// Custom Data test
	if (itest==1)
	{
	printf ("\tCustom data test:\n\n");
	inquire("\tCustom data 0000-FFFF  %4X", 0x0000,  0xFFFF, 16, scsi_wr_data);
	}

// Loop over all tests
	itest_first = itest;
	itest_last  = itest;

	if (itest==8 || itest==9) {itest_first=2; itest_last=7;}

	for (i=0; i<=23;++i) {
	adb_passed[i]=false;}

adb_loop:
	for (i=0; i<=alct_ntests; ++i) {alct_npassed[i]=0; alct_nfailed[i]=0; alct_nskipped[i]=0;}

	for (itest=itest_first; itest <=itest_last; ++itest)
	{

// Loop over passes
	for (ipass=1; ipass <=npasses; ++ipass)
	{

// Initialize test
	switch (itest)
	{
	case 1:	npatterns = 1;	scsi_wr_data = scsi_wr_data;	break;	// Custom data
	case 2:	npatterns = 16;	scsi_wr_data = 0x0000;			break;	// Running '1'
	case 3:	npatterns = 16;	scsi_wr_data = 0x0000;			break;	// Running '0'
	case 4:	npatterns = 16;	scsi_wr_data = 0x0000;			break;	// Filling by '1'
	case 5:	npatterns = 16;	scsi_wr_data = 0xFFFF;			break;	// Filling by '0'
	case 6:	npatterns = 16;	scsi_wr_data = 0xAAAA;			break;	// Shifting '5's and 'A's
	case 7:	npatterns = 16;	scsi_wr_data = 0x0000;			break;	// Random data
	}

// Loop over patterns
	for (ipattern=0; ipattern<npatterns; ++ipattern)
	{

// Next pattern
	switch (itest)
	{
	case 1:	scsi_wr_data =  scsi_wr_data;									break;	// Custom data
	case 2:	scsi_wr_data =  (0x1 << ipattern);								break;	// Running '1'
	case 3:	scsi_wr_data = ~(0x1 << ipattern) & 0xFFFF;						break;	// Running '0'
	case 4:	scsi_wr_data =  (scsi_wr_data |  (0x1 << ipattern));			break;	// Filling by '1'
	case 5:	scsi_wr_data =  (scsi_wr_data & ~(0x1 << ipattern)) & 0xFFFF;	break;	// Filling by '0's
	case 6:	scsi_wr_data = ~(scsi_wr_data) & 0xFFFF;;						break;	// Shifting '5's and 'A's
	case 7:	scsi_wr_data =  (rand() | (rand()<<15)) & 0xFFFF;				break;	// Random data (rand returns 0-0x7FFF)
	}

	iprint = (itest==itest_first) && (itest==itest_last);

// Write ADB channel
	if (adb_wr_ch<0 || adb_wr_ch>23) stop("\t adb_wr_ch out of range");

	opcode  = 0x16;											// ALCT opcode
	reg_len = 9;											// Register length

	i4_to_tdi(i4=adb_wr_ch,&tdi[0], 9,0);					// Convert integer to tdi bit array

	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi

// Clear TDI and TDO arrays
	for (i=0; i<mxbitstream; ++i) {tdi[i]=0; tdo[i];}

// Read back ADB channel
	opcode  = 0x15;											// ALCT opcode
	reg_len = 9;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],adb_rd_ch,reg_len,0);
	if (iprint) printf("\tADB  channel readback  %4.4X=%2.2i\n",adb_rd_ch, adb_rd_ch);

// Write SCSI data to tx on cable
	opcode  = 0x18;											// ALCT opcode
	reg_len = 16;											// Register length

	i4_to_tdi(i4=scsi_wr_data,&tdi[0], 16, 0);				// Convert integer to tdi bit array

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi

// Clear TDI and TDO arrays
	for (i=0; i<mxbitstream; ++i) {tdi[i]=0; tdo[i];}

// Read back SCSI data
	opcode  = 0x17;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],scsi_rd_data,reg_len,0);

	if (iprint) printf("\tSCSI tx data readback  %4.4X\n",scsi_rd_data);

// Write ADB channel ORd with 0x40 to set bit 6 in register 0x16
	opcode  = 0x16;											// ALCT opcode
	reg_len = 9;											// Register length

	i4_to_tdi(i4=(adb_wr_ch|0x40),&tdi[0], 9,0);			// Convert integer to tdi bit array

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi

// Write ADB channel to un-set bit 6 in register 0x16
	opcode  = 0x16;											// ALCT opcode
	reg_len = 9;											// Register length

	i4_to_tdi(i4=adb_wr_ch,&tdi[0], 9,0);					// Convert integer to tdi bit array

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi

// Read ADB data looped back via cable
adb_data_loop:

	opcode  = 0x19;											// ALCT opcode
	reg_len = 16;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],adb_rd_data,reg_len,0);

	if (adb_rd_data==scsi_wr_data) sok="OK";
	else                           sok="ERR";

	if (sok=="OK") 	alct_npassed[itest]++;
	if (sok=="ERR") alct_nfailed[itest]++;

	if (iprint)      printf("\tADB=%2i Cable read data %4.4X   %s\n",adb_wr_ch,adb_rd_data,sok.c_str());

	if (sok=="ERR") {printf("\n");
					 printf("\tADB=%2i Cable read data %4.4X   %s\n",adb_wr_ch,adb_rd_data,sok.c_str());
					 printf("\tADB=%2i Expected        %4.4X     \n",adb_wr_ch,scsi_wr_data);
					 printf("\tADB cable read data error: Skip Retry Exit [R]");
					 gets(line);
					 if (line[0]==NULL)					goto adb_data_loop;		// Default Retry
					 if (line[0]=='R' || line[0]=='r')	goto adb_data_loop;		// Retry
					 if (line[0]=='S' || line[0]=='s')	goto next_adb;			// Skip
					 if (line[0]=='E' || line[0]=='e')	goto L1400_menu;		// Exit
					}

// Read ADB hit list, only the selected ADB should have non-zero bits
adb_hit_loop:

	opcode  = 0x0A;											// ALCT opcode
	reg_len = 24;											// Register length

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read tdo

	tdi_to_i4(&tdo[0],adb_hit,reg_len,0);

	adb_hit_expect = (1<<adb_wr_ch);

	if       (adb_hit==adb_hit_expect)          sok="OK";	// ADB with hits should match selected ADB
	else if ((adb_hit==0) && (scsi_wr_data==0)) sok="OK";	// Unless the tx data is 0
	else                                        sok="ERR";	// Otherwise fail

	if (sok=="OK") 	alct_npassed[itest]++;
	if (sok=="ERR") alct_nfailed[itest]++;

	if (iprint)      printf("\tADB=%2i Connectors hit  %6.6X %s\n",adb_wr_ch,adb_hit,sok.c_str());\

	if (sok=="ERR") {printf("\n");
					 printf("\tADB=%2i Connectors hit  %6.6X %s\n",adb_wr_ch,adb_hit,sok.c_str());
	                 printf("\tADB=%2i Expected        %6.6X   \n",adb_wr_ch,adb_hit_expect);
		             printf("\tADB connector hit error: Skip Retry Exit [R]");
					 gets(line);
					 if (line[0]==NULL)					goto adb_hit_loop;		// Default Retry
					 if (line[0]=='R' || line[0]=='r')	goto adb_hit_loop;		// Retry
					 if (line[0]=='S' || line[0]=='s')	goto next_adb;			// Skip
					 if (line[0]=='E' || line[0]=='e')	goto L1400_menu;		// Exit
					}

// All-tests mode display
	if (!iprint)
	{
	printf("\tADB=%2i test=%1i pass=%2i sent=0x%4.4X received=0x%4.4X ADBlist=%6.6X expected=%6.6X\r",
	adb_wr_ch, itest, ipass, scsi_wr_data, adb_rd_data, adb_hit, adb_hit_expect);
	if (!adb_auto && alct_nfailed[itest]!=0) pause(" ERROR");
	}

// Close pattern loop
	}	// for ipattern
	}	// for ipass
	}	// for itest

	ifail=false;
	for (i=2; i<=7; ++i) if (alct_nfailed[i]!=0) ifail=true;

	if (!ifail) printf("\n\tADB=%2i passed\n",adb_wr_ch);
	else        printf("\n\tADB=%2i failed\n",adb_wr_ch);

// Automatic ADB increment
next_adb:

	if (adb_auto)
	{
	adb_passed[adb_wr_ch]=!ifail;

	Beep(784,250);	// G
	Beep(440,250);	// A
	Beep(698,250);	// F
	Beep(349,250);	// F octave lower
	Beep(523,250);	// C

// Next ADB
	adb_wr_ch++;

	if (adb_wr_ch<24)
	{
	printf("\n");
	printf("\tPress <CR> to advance to ADB channel=%2i Connector=%2i",adb_wr_ch,adb_wr_ch+1);
	gets(line);
	goto adb_loop;
	}

// ADB auto summary
	printf("\n");
	printf("\tADB Single Cable Test Summary:\n");

	for (i=0; i<24; ++i) {
	if (adb_passed[i]==true) printf("\tADB=%2i passed\n",i);
	else                     printf("\tADB=%2i failed\n",i);
	}
	npasses   = 1;
	adb_wr_ch = 0;
	goto L1400_menu;

	}	// close if adb_auto

// Done with tests
	goto L1400_menu;
}

//------------------------------------------------------------------------------
//	Slow Control Walking 1 Test
//------------------------------------------------------------------------------
	void L1500() {
// L1500:
	printf("\tSlow Control FPGA Walking 1 Test\n");

// Pins to skip: 0=expect driven value, 1=expect 0 from vcc or gnd, 2=input pin could be 0 or 1
	ipin_skip[1]	= 1;	ipin_name[1]	= "GND";				// GND	P1		GND
	ipin_skip[2]	= 2;	ipin_name[2]	= "tck_user";			// In	P2		I/O		PGCK1, GCK1, tck user
	ipin_skip[3]	= 0;	ipin_name[3]	= "nstandby[38]";		// Out	P3		I/O
	ipin_skip[4]	= 0;	ipin_name[4]	= "nstandby[39]";		// Out	P4		I/O
	ipin_skip[5]	= 0;	ipin_name[5]	= "nstandby[40]";		// Out	P5		I/O
	ipin_skip[6]	= 2;	ipin_name[6]	= "TDI";				// In	P6		I/O		TDI
	ipin_skip[7]	= 2;	ipin_name[7]	= "TCK";				// In	P7		I/O		TCK
	ipin_skip[8]	= 0;	ipin_name[8]	= "nstandby[41]";		// Out	P8		I/O
	ipin_skip[9]	= 2;	ipin_name[9]	= "tdi_user";			// In	P9		I/O
	ipin_skip[10]	= 2;	ipin_name[10]	= "tdo_user";			// Out	P10		I/O
	ipin_skip[11]	= 2;	ipin_name[11]	= "tms_user";			// In	P11		I/O
	ipin_skip[12]	= 0;	ipin_name[12]	= "unused[12]";			// Out	P12		I/O
	ipin_skip[13]	= 1;	ipin_name[13]	= "GND";				// GND	P13		GND
	ipin_skip[14]	= 0;	ipin_name[14]	= "unused[14]";			// Out	P14		I/O
	ipin_skip[15]	= 0;	ipin_name[15]	= "unused[15]";			// Out	P15		I/O
	ipin_skip[16]	= 2;	ipin_name[16]	= "TMS";				// In	P16		I/O		TMS
	ipin_skip[17]	= 0;	ipin_name[17]	= "unused[17]";			// Out	P17		I/O
	ipin_skip[18]	= 1;	ipin_name[18]	= "VCC";				// VCC	P18		VCC
	ipin_skip[19]	= 0;	ipin_name[19]	= "unused[19]";			// Out	P19		I/O 
	ipin_skip[20]	= 0;	ipin_name[20]	= "unused[20]";			// Out	P20		I/O
	ipin_skip[21]	= 0;	ipin_name[21]	= "unused[21]";			// Out	P21		I/O
	ipin_skip[22]	= 0;	ipin_name[22]	= "unused[22]";			// Out	P22		I/O
	ipin_skip[23]	= 0;	ipin_name[23]	= "unused[23]";			// Out	P23		I/O
	ipin_skip[24]	= 0;	ipin_name[24]	= "unused[24]";			// Out	P24		I/O
	ipin_skip[25]	= 1;	ipin_name[25]	= "GND";				// GND	P25		GND
	ipin_skip[26]	= 1;	ipin_name[26]	= "VCC";				// VCC	P26		VCC
	ipin_skip[27]	= 0;	ipin_name[27]	= "unused[27]";			// Out	P27		I/O
	ipin_skip[28]	= 0;	ipin_name[28]	= "unused[28]";			// Out	P28		I/O
	ipin_skip[29]	= 0;	ipin_name[29]	= "unused[29]";			// Out	P29		I/O
	ipin_skip[30]	= 0;	ipin_name[30]	= "unused[30]";			// Out	P30		I/O
	ipin_skip[31]	= 0;	ipin_name[31]	= "unused[31]";			// Out	P31		I/O
	ipin_skip[32]	= 0;	ipin_name[32]	= "unused[32]";			// Out	P32		I/O
	ipin_skip[33]	= 1;	ipin_name[33]	= "VCC";				// VCC	P33		VCC
	ipin_skip[34]	= 0;	ipin_name[34]	= "unused[34]";			// Out	P34		I/O
	ipin_skip[35]	= 0;	ipin_name[35]	= "unused[35]";			// Out	P35		I/O
	ipin_skip[36]	= 0;	ipin_name[36]	= "unused[36]";			// Out	P36		I/O
	ipin_skip[37]	= 0;	ipin_name[37]	= "unused[37]";			// Out	P37		I/O
	ipin_skip[38]	= 1;	ipin_name[38]	= "GND";				// GND	P38		GND
	ipin_skip[39]	= 0;	ipin_name[39]	= "unused[39]";			// Out	P39		I/O
	ipin_skip[40]	= 0;	ipin_name[40]	= "unused[40]";			// Out	P40		I/O
	ipin_skip[41]	= 0;	ipin_name[41]	= "state[0]";			// Out	P41		I/O
	ipin_skip[42]	= 0;	ipin_name[42]	= "state[1]";			// Out	P42		I/O
	ipin_skip[43]	= 0;	ipin_name[43]	= "state[2]";			// Out	P43		I/O
	ipin_skip[44]	= 0;	ipin_name[44]	= "state[3]";			// Out	P44		I/O
	ipin_skip[45]	= 0;	ipin_name[45]	= "opcode[0]";			// Out	P45		I/O
	ipin_skip[46]	= 0;	ipin_name[46]	= "opcode[1]";			// Out	P46		I/O
	ipin_skip[47]	= 0;	ipin_name[47]	= "opcode[2]";			// Out	P47		I/O
	ipin_skip[48]	= 0;	ipin_name[48]	= "opcode[3]";			// Out	P48		I/O
	ipin_skip[49]	= 0;	ipin_name[49]	= "unused[49]";			// Out	P49		I/O		SGCK2, GCK2
	ipin_skip[50]	= 2;	ipin_name[50]	= "M1";					// In	P50		M1
	ipin_skip[51]	= 1;	ipin_name[51]	= "GND";				// GND	P51		GND
	ipin_skip[52]	= 2;	ipin_name[52]	= "M0";					// In	P52		M0
	ipin_skip[53]	= 1;	ipin_name[53]	= "VCC";				// VCC	P53		VCC
	ipin_skip[54]	= 2;	ipin_name[54]	= "PWWDN";				// In	P54		PWRDWN
	ipin_skip[55]	= 0;	ipin_name[55]	= "unused[55]";			// Out	P55		I/O		PGCK2, GCK3
	ipin_skip[56]	= 0;	ipin_name[56]	= "unused[56]";			// Out	P56		HDC
	ipin_skip[57]	= 0;	ipin_name[57]	= "opcode[4]";			// Out	P57		I/O
	ipin_skip[58]	= 0;	ipin_name[58]	= "opcode[5]";			// Out	P58		I/O
	ipin_skip[59]	= 0;	ipin_name[59]	= "future[0]";			// Out	P59		I/O
	ipin_skip[60]	= 0;	ipin_name[60]	= "unused[60]";			// Out	P60		I/O		LDC
	ipin_skip[61]	= 0;	ipin_name[61]	= "future[1]";			// Out	P61		I/O
	ipin_skip[62]	= 0;	ipin_name[62]	= "future[2]";			// Out	P62		I/O
	ipin_skip[63]	= 0;	ipin_name[63]	= "future[3]";			// Out	P63		I/O
	ipin_skip[64]	= 0;	ipin_name[64]	= "future[4]";			// Out	P64		I/O
	ipin_skip[65]	= 0;	ipin_name[65]	= "unused[65]";			// Out	P65		I/O
	ipin_skip[66]	= 1;	ipin_name[66]	= "GND";				// GND	P66		GND
	ipin_skip[67]	= 0;	ipin_name[67]	= "future[5]";			// Out	P67		I/O
	ipin_skip[68]	= 0;	ipin_name[68]	= "future[6]";			// Out	P68		I/O
	ipin_skip[69]	= 0;	ipin_name[69]	= "future[7]";			// Out	P69		I/O
	ipin_skip[70]	= 0;	ipin_name[70]	= "din_adc[0]";			// Out	P70		I/O
	ipin_skip[71]	= 1;	ipin_name[71]	= "VCC";				// VCC	P71		VCC
	ipin_skip[72]	= 2;	ipin_name[72]	= "dout_adc[0]";		// In	P72		I/O
	ipin_skip[73]	= 0;	ipin_name[73]	= "ncs_adc[0]";			// Out	P73		I/O
	ipin_skip[74]	= 0;	ipin_name[74]	= "clk_adc[0]";			// Out	P74		I/O
	ipin_skip[75]	= 0;	ipin_name[75]	= "din_adc[1]";			// Out	P75		I/O
	ipin_skip[76]	= 2;	ipin_name[76]	= "dout_adc[1]";		// In	P76		I/O
	ipin_skip[77]	= 2;	ipin_name[77]	= "INIT";				// Out	P77		I/O		INIT
	ipin_skip[78]	= 1;	ipin_name[78]	= "VCC";				// VCC	P78		VCC
	ipin_skip[79]	= 1;	ipin_name[79]	= "GND";				// GND	P79		GND
	ipin_skip[80]	= 0;	ipin_name[80]	= "ncs_adc[1]";			// Out	P80		I/O
	ipin_skip[81]	= 0;	ipin_name[81]	= "clk_adc[1]";			// Out	P81		I/O
	ipin_skip[82]	= 0;	ipin_name[82]	= "din_adc[2]";			// Out	P82		I/O
	ipin_skip[83]	= 2;	ipin_name[83]	= "dout_adc[2]";		// In	P83		I/O
	ipin_skip[84]	= 0;	ipin_name[84]	= "ncs_adc[2]";			// Out	P84		I/O
	ipin_skip[85]	= 0;	ipin_name[85]	= "clk_adc[2]";			// Out	P85		I/O
	ipin_skip[86]	= 1;	ipin_name[86]	= "VCC";				// VCC	P86		VCC
	ipin_skip[87]	= 0;	ipin_name[87]	= "din_adc[3]";			// Out	P87		I/O
	ipin_skip[88]	= 2;	ipin_name[88]	= "dout_adc[3]";		// In	P88		I/O
	ipin_skip[89]	= 0;	ipin_name[89]	= "ncs_adc[3]";			// Out	P89		I/O
	ipin_skip[90]	= 0;	ipin_name[90]	= "clk_adc[3]";			// Out	P90		I/O
	ipin_skip[91]	= 1;	ipin_name[91]	= "GND";				// GND	P91		GND
	ipin_skip[92]	= 0;	ipin_name[92]	= "unused[92]";			// Out	P92		I/O
	ipin_skip[93]	= 0;	ipin_name[93]	= "din_adc[4]";			// Out	P93		I/O
	ipin_skip[94]	= 2;	ipin_name[94]	= "dout_adc[4]";		// In	P94		I/O
	ipin_skip[95]	= 0;	ipin_name[95]	= "ncs_adc[4]";			// Out	P95		I/O
	ipin_skip[96]	= 0;	ipin_name[96]	= "clk_adc[4]";			// Out	P96		I/O
	ipin_skip[97]	= 0;	ipin_name[97]	= "din_dac[0]";			// Out	P97		I/O
	ipin_skip[98]	= 0;	ipin_name[98]	= "nrs_dac[0]";			// Out	P98		I/O
	ipin_skip[99]	= 0;	ipin_name[99]	= "ncs_dac[0]";			// Out	P99		I/O
	ipin_skip[100]	= 0;	ipin_name[100]	= "clk_dac[0]";			// Out	P100	I/O
	ipin_skip[101]	= 0;	ipin_name[101]	= "din_dac[1]";			// Out	P101	I/O
	ipin_skip[102]	= 0;	ipin_name[102]	= "unused[102]";		// Out	P102	I/O		SGCK3, GCK4
	ipin_skip[103]	= 1;	ipin_name[103]	= "GND";				// GND	P103	GND
	ipin_skip[104]	= 2;	ipin_name[104]	= "DONE";				// In	P104	DONE
	ipin_skip[105]	= 1;	ipin_name[105]	= "VCC";				// VCC	P105	VCC
	ipin_skip[106]	= 2;	ipin_name[106]	= "PROGRAM";			// In	P106	PROGRAM 
	ipin_skip[107]	= 0;	ipin_name[107]	= "unused[107]";		// Out	P107	I/O		D7
	ipin_skip[108]	= 0;	ipin_name[108]	= "unused[108]";		// Out	P108	I/O		PGCK3, GCK5
	ipin_skip[109]	= 0;	ipin_name[109]	= "nrs_dac[1]";			// Out	P109	I/O
	ipin_skip[110]	= 0;	ipin_name[110]	= "ncs_dac[1]";			// Out	P110	I/O
	ipin_skip[111]	= 0;	ipin_name[111]	= "unused[111]";		// Out	P111	I/O
	ipin_skip[112]	= 0;	ipin_name[112]	= "unused[112]";		// Out	P112	I/O		D6
	ipin_skip[113]	= 0;	ipin_name[113]	= "clk_dac[1]";			// Out	P113	I/O
	ipin_skip[114]	= 0;	ipin_name[114]	= "din_dac[2]";			// Out	P114	I/O
	ipin_skip[115]	= 0;	ipin_name[115]	= "nrs_dac[2]";			// Out	P115	I/O
	ipin_skip[116]	= 0;	ipin_name[116]	= "ncs_dac[2]";			// Out	P116	I/O
	ipin_skip[117]	= 0;	ipin_name[117]	= "clk_dac[2]";			// Out	P117	I/O
	ipin_skip[118]	= 1;	ipin_name[118]	= "GND";				// GND	P118	GND
	ipin_skip[119]	= 0;	ipin_name[119]	= "din_dac[3]";			// Out	P119	I/O
	ipin_skip[120]	= 0;	ipin_name[120]	= "nrs_dac[3]";			// Out	P120	I/O
	ipin_skip[121]	= 1;	ipin_name[121]	= "VCC";				// VCC	P121	VCC
	ipin_skip[122]	= 0;	ipin_name[122]	= "unused[122]";		// Out	P122	I/O		D5
	ipin_skip[123]	= 0;	ipin_name[123]	= "ncs_dac[3]";			// Out	P123	I/O
	ipin_skip[124]	= 0;	ipin_name[124]	= "clk_dac[3]";			// Out	P124	I/O
	ipin_skip[125]	= 0;	ipin_name[125]	= "din_tp";				// Out	P125	I/O
	ipin_skip[126]	= 0;	ipin_name[126]	= "unused[126]";		// Out	P126	I/O
	ipin_skip[127]	= 0;	ipin_name[127]	= "clk_tp";				// Out	P127	I/O
	ipin_skip[128]	= 0;	ipin_name[128]	= "unused[128]";		// Out	P128	I/O		D4
	ipin_skip[129]	= 0;	ipin_name[129]	= "ncs_tp";				// Out	P129	I/O
	ipin_skip[130]	= 1;	ipin_name[130]	= "VCC";				// VCC	P130	VCC
	ipin_skip[131]	= 1;	ipin_name[131]	= "GND";				// GND	P131	GND
	ipin_skip[132]	= 0;	ipin_name[132]	= "unused[132]";		// Out	P132	I/O		D3
	ipin_skip[133]	= 0;	ipin_name[133]	= "npd_tp";				// Out	P133	I/O
	ipin_skip[134]	= 0;	ipin_name[134]	= "tp_strip_en[0]";		// Out	P134	I/O
	ipin_skip[135]	= 0;	ipin_name[135]	= "tp_strip_en[1]";		// Out	P135	I/O
	ipin_skip[136]	= 0;	ipin_name[136]	= "tp_strip_en[2]";		// Out	P136	I/O
	ipin_skip[137]	= 0;	ipin_name[137]	= "tp_strip_en[3]";		// Out	P137	I/O
	ipin_skip[138]	= 0;	ipin_name[138]	= "unused[138]";		// Out	P138	I/O		D2
	ipin_skip[139]	= 0;	ipin_name[139]	= "tp_strip_en[4]";		// Out	P139	I/O
	ipin_skip[140]	= 1;	ipin_name[140]	= "VCC";				// VCC	P140	VCC
	ipin_skip[141]	= 0;	ipin_name[141]	= "tp_strip_en[5]";		// Out	P141	I/O
	ipin_skip[142]	= 0;	ipin_name[142]	= "tp_group_en[0]";		// Out	P142	I/O
	ipin_skip[143]	= 1;	ipin_name[143]	= "GND";				// GND	P143	GND
	ipin_skip[144]	= 0;	ipin_name[144]	= "unused[144]";		// Out	P144	I/O
	ipin_skip[145]	= 0;	ipin_name[145]	= "tp_group_en[1]";		// Out	P145	I/O
	ipin_skip[146]	= 0;	ipin_name[146]	= "tp_group_en[2]";		// Out	P146	I/O
	ipin_skip[147]	= 0;	ipin_name[147]	= "tp_group_en[3]";		// Out	P147	I/O
	ipin_skip[148]	= 0;	ipin_name[148]	= "tp_group_en[4]";		// Out	P148	I/O
	ipin_skip[149]	= 0;	ipin_name[149]	= "unused[149]";		// Out	P149	I/O		D1
	ipin_skip[150]	= 0;	ipin_name[150]	= "tp_group_en[5]";		// Out	P150	I/O
	ipin_skip[151]	= 0;	ipin_name[151]	= "tp_group_en[6]";		// Out	P151	I/O
	ipin_skip[152]	= 0;	ipin_name[152]	= "nstandby[0]";		// Out	P152	I/O
	ipin_skip[153]	= 0;	ipin_name[153]	= "unused[153]";		// Out	P153	I/O		D0, DIN
	ipin_skip[154]	= 0;	ipin_name[154]	= "unused[154]";		// Out	P154	I/O		SGCK4, GCK6, DOUT
	ipin_skip[155]	= 2;	ipin_name[155]	= "CCK";				// In	P155	CCK
	ipin_skip[156]	= 1;	ipin_name[156]	= "VCC";				// VCC	P156	VCC
	ipin_skip[157]	= 2;	ipin_name[157]	= "TDO";				// In	P157	TDO
	ipin_skip[158]	= 1;	ipin_name[158]	= "GND";				// GND	P158	GND
	ipin_skip[159]	= 0;	ipin_name[159]	= "nstandby[1]";		// Out	P159	I/O
	ipin_skip[160]	= 0;	ipin_name[160]	= "unused[160]";		// Out	P160	I/O		PGCK4, GCK7
	ipin_skip[161]	= 0;	ipin_name[161]	= "nstandby[2]";		// Out	P161	I/O
	ipin_skip[162]	= 0;	ipin_name[162]	= "nstandby[3]";		// Out	P162	I/O
	ipin_skip[163]	= 2;	ipin_name[163]	= "CS1";				// In	P163	I/O		CS1
	ipin_skip[164]	= 0;	ipin_name[164]	= "nstandby[4]";		// Out	P164	I/O
	ipin_skip[165]	= 0;	ipin_name[165]	= "unused[165]";		// Out	P165	I/O
	ipin_skip[166]	= 0;	ipin_name[166]	= "nstandby[5]";		// Out	P166	I/O
	ipin_skip[167]	= 0;	ipin_name[167]	= "nstandby[6]";		// Out	P167	I/O
	ipin_skip[168]	= 0;	ipin_name[168]	= "nstandby[7]";		// Out	P168	I/O
	ipin_skip[169]	= 0;	ipin_name[169]	= "nstandby[8]";		// Out	P169	I/O
	ipin_skip[170]	= 1;	ipin_name[170]	= "GND";				// GND	P170	GND
	ipin_skip[171]	= 0;	ipin_name[171]	= "nstandby[9]";		// Out	P171	I/O
	ipin_skip[172]	= 0;	ipin_name[172]	= "nstandby[10]";		// Out	P172	I/O
	ipin_skip[173]	= 1;	ipin_name[173]	= "VCC";				// VCC	P173	VCC
	ipin_skip[174]	= 0;	ipin_name[174]	= "nstandby[11]";		// Out	P174	I/O
	ipin_skip[175]	= 0;	ipin_name[175]	= "nstandby[12]";		// Out	P175	I/O
	ipin_skip[176]	= 0;	ipin_name[176]	= "nstandby[13]";		// Out	P176	I/O
	ipin_skip[177]	= 0;	ipin_name[177]	= "nstandby[14]";		// Out	P177	I/O
	ipin_skip[178]	= 0;	ipin_name[178]	= "nstandby[15]";		// Out	P178	I/O
	ipin_skip[179]	= 0;	ipin_name[179]	= "nstandby[16]";		// Out	P179	I/O
	ipin_skip[180]	= 0;	ipin_name[180]	= "nstandby[17]";		// Out	P180	I/O
	ipin_skip[181]	= 0;	ipin_name[181]	= "nstandby[18]";		// Out	P181	I/O
	ipin_skip[182]	= 1;	ipin_name[182]	= "GND";				// GND	P182	GND
	ipin_skip[183]	= 1;	ipin_name[183]	= "VCC";				// VCC	P183	VCC
	ipin_skip[184]	= 0;	ipin_name[184]	= "nstandby[19]";		// Out	P184	I/O
	ipin_skip[185]	= 0;	ipin_name[185]	= "nstandby[20]";		// Out	P185	I/O
	ipin_skip[186]	= 0;	ipin_name[186]	= "nstandby[21]";		// Out	P186	I/O
	ipin_skip[187]	= 0;	ipin_name[187]	= "nstandby[22]";		// Out	P187	I/O
	ipin_skip[188]	= 0;	ipin_name[188]	= "nstandby[23]";		// Out	P188	I/O
	ipin_skip[189]	= 0;	ipin_name[189]	= "nstandby[24]";		// Out	P189	I/O
	ipin_skip[190]	= 0;	ipin_name[190]	= "nstandby[25]";		// Out	P190	I/O
	ipin_skip[191]	= 0;	ipin_name[191]	= "nstandby[26]";		// Out	P191	I/O
	ipin_skip[192]	= 1;	ipin_name[192]	= "VCC";				// VCC	P192	VCC
	ipin_skip[193]	= 0;	ipin_name[193]	= "nstandby[27]";		// Out	P193	I/O
	ipin_skip[194]	= 0;	ipin_name[194]	= "nstandby[28]";		// Out	P194	I/O
	ipin_skip[195]	= 1;	ipin_name[195]	= "GND";				// GND	P195	GND
	ipin_skip[196]	= 0;	ipin_name[196]	= "nstandby[29]";		// Out	P196	I/O
	ipin_skip[197]	= 0;	ipin_name[197]	= "nstandby[30]";		// Out	P197	I/O
	ipin_skip[198]	= 0;	ipin_name[198]	= "nstandby[31]";		// Out	P198	I/O
	ipin_skip[199]	= 0;	ipin_name[199]	= "nstandby[32]";		// Out	P199	I/O
	ipin_skip[200]	= 0;	ipin_name[200]	= "nstandby[33]";		// Out	P200	I/O
	ipin_skip[201]	= 0;	ipin_name[201]	= "nstandby[34]";		// Out	P201	I/O
	ipin_skip[202]	= 0;	ipin_name[202]	= "unused[202]";		// Out	P202	I/O
	ipin_skip[203]	= 0;	ipin_name[203]	= "unused[203]";		// Out	P203	I/O
	ipin_skip[204]	= 0;	ipin_name[204]	= "nstandby[35]";		// Out	P204	I/O
	ipin_skip[205]	= 0;	ipin_name[205]	= "nstandby[36]";		// Out	P205	I/O
	ipin_skip[206]	= 0;	ipin_name[206]	= "nstandby[37]";		// Out	P206	I/O
	ipin_skip[207]	= 0;	ipin_name[207]	= "unused[207]";		// Out	P207	I/O		SGCK1, GCK8
	ipin_skip[208]	= 1;	ipin_name[208]	= "VCC";				// VCC	P208	VCC

// Create fat 0 for writing to data registers
	for (i=0; i<mxbitstream; ++i) tdi[i]=0;

// Clear walking 1 register in Slow Control FPGA
	adr     = boot_adr;										// Boot register address
	ichain  = 0x00;											// ALCT Mezzanine control user jtag chain
	chip_id = 0;											// ALCT slow control user jtag path has 1 chips
	reg_len = 216;											// Register length

	opcode = 0x02;											// Write IO pins
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

// Read FPGA IO pins
	opcode = 0x01;											// Read IO pins
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	tdi_to_i4(&tdo[  0],sc_begin,4,0);
	tdi_to_i4(&tdo[212],sc_end  ,4,0);

// Store initial IO pin state
	for (ipin=1; ipin<=208; ++ipin) ipin_init[ipin]=tdo[ipin+3];// ipin=1 is tdo[4]

	zeros = count0s(&ipin_init[1],i=208);
	ones  = count1s(&ipin_init[1],i=208);
	nshorted = 0;

	fprintf(stdout,"Initial State\n");
	fprintf(stdout,"%1.1X\n",sc_begin);
	for (i=  1; i<= 52; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	for (i= 53; i<=104; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	for (i=105; i<=156; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	for (i=157; i<=208; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	fprintf(stdout,"%1.1X\n",sc_end);
	fprintf(stdout,"zeros = %3i\n",zeros);
	fprintf(stdout,"ones  = %3i\n",ones );
	fprintf(stdout,"\n");

// Check that all IOs have pulled down
	for (ipin=1; ipin<=208; ++ipin)
	{
	iexpect=-1;								// type 2 can read back 0 or 1
	if (ipin_skip[ipin]==0) iexpect = 0;	// IO pin should be pulled down
	if (ipin_skip[ipin]==1) iexpect = 0;	// VCC or GND pin should always read back 0

	if (iexpect>=0 && ipin_init[ipin]!=iexpect) 
	{
	fprintf(stdout,"Pin %3i %s expected %1i but read %1i\n",ipin,ipin_name[ipin].c_str(),iexpect,ipin_init[ipin]);
	pause("Fix this before you continue");
	}
	}

// Walk a 1 though the IO pins
	for (ipin=1; ipin<=208; ++ipin)
	{
	for (i=1; i<=208; ++i) tdi[i+3]=0; tdi[ipin+3]=1;		// ipin=1 is tdo[4]

	opcode = 0x02;											// Write walking one pattern
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	opcode = 0x01;											// Read IO pins
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	tdi_to_i4(&tdo[  0],sc_begin,4,0);
	tdi_to_i4(&tdo[212],sc_end  ,4,0);

	for (i=1; i<=208; ++i) ipin_state[i]=tdo[i+3];			// Copy pin state

	zeros = count0s(&ipin_state[1],i=208);
	ones  = count1s(&ipin_state[1],i=208);

	fprintf(stdout,"Set pin %3i to 1  %s  skip=%1i\n",ipin,ipin_name[ipin].c_str(),ipin_skip[ipin]);
	fprintf(stdout,"%1.1X\n",sc_begin);
	for (i=  1; i<= 52; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	for (i= 53; i<=104; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	for (i=105; i<=156; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	for (i=157; i<=208; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	fprintf(stdout,"%1.1X\n",sc_end);
	fprintf(stdout,"zeros = %3i\n",zeros);
	fprintf(stdout,"ones  = %3i\n",ones );
	fprintf(stdout,"\n");

// See if driven pin went high
	if (ipin_state[ipin]==0 && ipin_skip[ipin]==0)
	{
	fprintf(stdout,"Driving pin %3i %s: Expected %1i but found %1i\n",ipin,ipin_name[ipin].c_str(),1,ipin_state[ipin]);
	pause("woe is me...woe is me");
	}

// Compare current pin state to initial pin state
	for (i=1; i<=208; ++i)
	{
	if (i==ipin) continue;				// Skip pin thats supposed to have a 1
	if (ipin_state[i] != ipin_init[i])
	{
	fprintf(stdout,"Driving pin %3i %s: Expected %1i but found %1i at pin=%3i\n",ipin,ipin_name[ipin].c_str(),ipin_init[i],ipin_state[i],i);
	nshorted++;
	ipin_driven[nshorted]  = ipin;
	ipin_shorted[nshorted] = i;
	pause("woe is me...woe is me");
	}
	}

// Close walking 1
	}
	fprintf(stdout,"\n");
	fprintf(stdout,"Shorted pins = %3i\n",nshorted);

	if (nshorted!=0)
	{
	for (i=1; i<=nshorted; ++i)
	{
	fprintf(stdout,"Short %3i: driven pin=%3i %s   shorted pin=%3i %s\n",
	i,
	ipin_driven[i],  ipin_name[ipin_driven[i]].c_str(),
	ipin_shorted[i], ipin_name[ipin_shorted[i]].c_str());
	}
	}


// TEMPSHIT FOLLOWS


// Load 1s into walking 0 register
	for (i=0; i<mxbitstream; ++i) tdi[i]=1;

	adr     = boot_adr;										// Boot register address
	ichain  = 0x00;											// ALCT Mezzanine control user jtag chain
	chip_id = 0;											// ALCT slow control user jtag path has 1 chips
	reg_len = 216;											// Register length

	opcode = 0x02;											// Write IO pins
	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

// Read FPGA IO pins
	opcode = 0x01;											// Read IO pins
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	tdi_to_i4(&tdo[  0],sc_begin,4,0);
	tdi_to_i4(&tdo[212],sc_end  ,4,0);

// Store initial IO pin state
	for (ipin=1; ipin<=208; ++ipin) ipin_init[ipin]=tdo[ipin+3];// ipin=1 is tdo[4]

	zeros = count0s(&ipin_init[1],i=208);
	ones  = count1s(&ipin_init[1],i=208);
	nshorted = 0;

	fprintf(stdout,"Initial State\n");
	fprintf(stdout,"%1.1X\n",sc_begin);
	for (i=  1; i<= 52; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	for (i= 53; i<=104; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	for (i=105; i<=156; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	for (i=157; i<=208; ++i) fprintf(stdout,"%1.1i",ipin_init[i]);	fprintf(stdout,"\n");
	fprintf(stdout,"%1.1X\n",sc_end);
	fprintf(stdout,"zeros = %3i\n",zeros);
	fprintf(stdout,"ones  = %3i\n",ones );
	fprintf(stdout,"\n");

// Check that all IOs have pulled up
	for (ipin=1; ipin<=208; ++ipin)
	{
	iexpect=-1;								// type 2 can read back 0 or 1
	if (ipin_skip[ipin]==0) iexpect = 1;	// IO pin should be pulled up
	if (ipin_skip[ipin]==1) iexpect = 0;	// VCC or GND pin should always read back 0

	if (iexpect>=0 && ipin_init[ipin]!=iexpect) 
	{
	fprintf(stdout,"Pin %3i %s expected %1i but read %1i\n",ipin,ipin_name[ipin].c_str(),iexpect,ipin_init[ipin]);
	pause("Fix this before you continue");
	}
	}

// Walk a 0 though the IO pins
	for (ipin=1; ipin<=208; ++ipin)
	{
	for (i=1; i<=208; ++i) tdi[i+3]=1; tdi[ipin+3]=0;		// ipin=1 is tdo[4]

	opcode = 0x02;											// Write walking one pattern
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	opcode = 0x01;											// Read IO pins
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	tdi_to_i4(&tdo[  0],sc_begin,4,0);
	tdi_to_i4(&tdo[212],sc_end  ,4,0);

	for (i=1; i<=208; ++i) ipin_state[i]=tdo[i+3];			// Copy pin state

	zeros = count0s(&ipin_state[1],i=208);
	ones  = count1s(&ipin_state[1],i=208);

	fprintf(stdout,"Set pin %3i to 0  %s  skip=%1i\n",ipin,ipin_name[ipin].c_str(),ipin_skip[ipin]);
	fprintf(stdout,"%1.1X\n",sc_begin);
	for (i=  1; i<= 52; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	for (i= 53; i<=104; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	for (i=105; i<=156; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	for (i=157; i<=208; ++i) fprintf(stdout,"%1.1i",ipin_state[i]);	fprintf(stdout,"\n");
	fprintf(stdout,"%1.1X\n",sc_end);
	fprintf(stdout,"zeros = %3i\n",zeros);
	fprintf(stdout,"ones  = %3i\n",ones );
	fprintf(stdout,"\n");

// See if driven pin went low
	if (ipin_state[ipin]==1 && ipin_skip[ipin]==0)
	{
	fprintf(stdout,"Driving pin %3i %s: Expected %1i but found %1i\n",ipin,ipin_name[ipin].c_str(),0,ipin_state[ipin]);
	pause("woe is me...woe is me");
	}

// Compare current pin state to initial pin state
	for (i=1; i<=208; ++i)
	{
	if (i==ipin) continue;				// Skip pin thats supposed to have a 1
	if (ipin_state[i] != ipin_init[i])
	{
	fprintf(stdout,"Driving pin %3i %s: Expected %1i but found %1i at pin=%3i\n",ipin,ipin_name[ipin].c_str(),ipin_init[i],ipin_state[i],i);
	nshorted++;
	ipin_driven[nshorted]  = ipin;
	ipin_shorted[nshorted] = i;
	pause("woe is me...woe is me");
	}
	}

// Close walking 0
	}
	fprintf(stdout,"\n");
	fprintf(stdout,"Shorted pins = %3i\n",nshorted);

	if (nshorted!=0)
	{
	for (i=1; i<=nshorted; ++i)
	{
	fprintf(stdout,"Short %3i: driven pin=%3i %s   shorted pin=%3i %s\n",
	i,
	ipin_driven[i],  ipin_name[ipin_driven[i]].c_str(),
	ipin_shorted[i], ipin_name[ipin_shorted[i]].c_str());
	}
	}

// Done
	fprintf(stdout,"\n");
	return;
	}

//------------------------------------------------------------------------------
//	Slow Control Toggle 1 Bit
//------------------------------------------------------------------------------
	void L1600() {
L1600:
	printf("\tSlow Control Toggle 1 Bit\n");
	printf("\tEnter pin to toggle [208:1]");

	gets(line);
	if (line[0]==NULL) goto L1600;
	sscanf(line,"%i",&ipin);
	if (ipin<1 || ipin>208) goto L1600;

	printf("\tToggling pin %3i forever: press any key to exit\n",ipin);

// Create fat 0 for writing to data registers
	for (i=0; i<mxbitstream; ++i) tdi[i]=0;

// Initialize jtag chain
	adr     = boot_adr;										// Boot register address
	ichain  = 0x00;											// ALCT Mezzanine control user jtag chain
	chip_id = 0;											// ALCT slow control user jtag path has 1 chips
	reg_len = 216;											// Register length

	vme_jtag_anystate_to_rti(adr,ichain);					// Take TAP to RTI

// Set ipin to 1
toggle:
	tdi[ipin+3]=1;

	opcode = 0x02;											// Write pin register
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

// Set ipin to 0
	tdi[ipin+3]=0;

	opcode = 0x02;											// Write pin register
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write tdi, read tdo

	if (!_kbhit()) goto toggle;								// Check for keyboard hit
	ckey = _getch();										// Read key to clear input buffer

// Done
	fprintf(stdout,"\n");
	return;
	}

//------------------------------------------------------------------------------
//	ALCT Test Sub-Menu
//------------------------------------------------------------------------------
	void L2300() {
L2300:

// Display menu
	printf("\n");
	printf("\tALCT Test Submenu:\n");
	printf("\t1:  Read ALCT JTAG Register:  NORMAL ALCT firmware\n");
	printf("\t2:  Read ALCT JTAG Register:  DEBUG  ALCT firmware\n");
	printf("\t8:  JTAG tests\n");
	printf("\t<cr> Exit\n");
	printf("       > ");

	gets(line);
	if (line[0]==NULL) return;
	sscanf(line,"%i",&ifunc);

	i=abs(ifunc);
	if (i== 1) goto L23100;
	if (i== 2) goto L23200;
	if (i== 8) goto L23800;
	goto L2300;

//------------------------------------------------------------------------------
//	ALCT: Read JTAG chain: Normal Firmware
//------------------------------------------------------------------------------
L23100:
	printf("\tMake sure you removed the JTAG cable\n");

//	Chain ID	Section		 Control or Program
//	--------	------------ ------------------
//	  0			Slow Control control registers
//	  1			Slow Control PROM
//	  2			Mezzanine    control registers
//	  3			Mezzanine    FPGA+PROM
//
//
//	Mezzanie Virtex Control Registers (5-bit opcode)
//
//	Name			OpCd		Len	Dir		Function
//	------------	---			---	-----	------------------
	IDRead        = 0x0;  // 	40	read	Virtex ID register
	HCMaskRead    = 0x1;  // 	384	read	hot mask
	HCMaskWrite   = 0x2;  // 	384	write	hot mask
	RdTrig        = 0x3;  // 	5	read	trigger register
	WrTrig        = 0x4;  // 	5	write	trigger register
	RdCfg         = 0x6;  // 	69	read	control register
	WrCfg         = 0x7;  // 	69	write	control register
	Wdly          = 0xd;  // 	120	write	delay lines. cs_dly bits in Par
	Rdly          = 0xe;  // 	121?read	delay lines. cs_dly bits in Par
	CollMaskRead  = 0x13; // 	224	read	collision pattern mask
	CollMaskWrite = 0x14; // 	224	write	collision pattern mask
	ParamRegRead  = 0x15; // 	6	read	delay line control register actually
	ParamRegWrite = 0x16; // 	6	read	delay line control register actually
	InputEnable   = 0x17; // 	0	write?	commands to disable and enable input
	InputDisable  = 0x18; // 	0	write?	commands to disable and enable input
	YRwrite       = 0x19; // 	31	write	output register (for debugging with UCLA test board)
	OSread        = 0x1a; // 	49	read	output storage
	SNread        = 0x1b; //	1	read	one bit of serial number
	SNwrite0      = 0x1c; //	0	write	0 bit into serial number chip
	SNwrite1      = 0x1d; //	0	write	1 bit into serial number chip
	SNreset       = 0x1e; //	0	write	reset serial number chip
	Bypass        = 0x1f; // 	1	bypass
//
//	Configuration Register
//	Register Bits	Signal				Default	BeamTest
//	-------------	--------------		-------	--------
//	ConfgReg[1:0]	trig_mode[1:0]		0		2
//	ConfgReg[2]		ext_trig_en			0		0
//	ConfgReg[3]		pretrig_halt		0		0
//	ConfgReg[4]		inject				0		?
//	ConfgReg[5]		inject_mode			0		?
//	ConfgReg[12:6]	inject_mask[6:0]	7Fh		?
//	ConfgReg[15:13]	nph_thresh[2:0]		2		2
//	ConfgReg[18:16]	nph_pattern[2:0]	4		4
//	ConfgReg[20:19]	drift_delay[1:0]	3		?
//	ConfgReg[25:21]	fifo_tbins[4:0]		7		8
//	ConfgReg[30:26]	fifo_pretrig[4:0]	1		12d
//	ConfgReg[32:31]	fifo_mode[1:0]		1		?
//	ConfgReg[35:33]	fifo_lastlct[2:0]	3		?
//	ConfgReg[43:36]	l1a_delay[7:0]		78h		128d, 78h=120d
//	ConfgReg[47:44]	l1a_window[3:0]		3		3
//	ConfgReg[51:48]	l1a_offset[3:0]		0		1
//	ConfgReg[52]	l1a_internal		0		0
//	ConfgReg[55:53]	BoardID[2:0]		5		?
//	ConfgReg[59:56]	bxn_offset[3:0]		0		?
//	ConfgReg[60]	ccb_enable			0		-
//	ConfgReg[61]	alct_jtag_ds		1		-
//	ConfgReg[63:62]	alct_tmode[1:0]		0		-
//	ConfgReg[65:64]	alct_amode[1:0]		0		?		
//	ConfgReg[66]	alct_mask_all		0		-
//	ConfgReg[67]	trig_info_en		1		?
//	ConfgReg[68]	sn_select			0		0
//
//
//	Virtex ID register
//	Field		Len	Typical	Description
//	-------		---	-------	--------------------------
//	[3:0]		4	7		Chip ID number, fixed at 7
//	[7:4]		4	C		Software Version ID [0-F]
//	[23:8]		16	2001	Year: 4 BCD digits
//	[31:24]		8	17		Day:  2 BCD digits
//	[39:32]		8	09		Month: 2 BCD digits
//
//------------------------------------------------------------------------------
// Select ALCT Mezzanine FPGA programming JTAG chain from TMB boot register
	ichain = 0x0003;								// ALCT Mezzanine pgm jtag chain
	adr    = boot_adr;								// Boot register address
	vme_jtag_anystate_to_rti(adr,ichain);			// Take TAP to RTI

// Read Virtex-E FPGA (5-bit opcode) and XC18V04 PROM IDcodes (8-bit opcode)
	for (chip_id=0; chip_id<=1; ++chip_id) {

	if (chip_id==0) opcode=0x09;					// FPGA IDcode opcode, expect v0A30093
	if (chip_id==1) opcode=0xFE;					// PROM IDcode opcode
	reg_len=32;										// IDcode length
													// FPGA,PROM chip
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],idcode,32,0);
	printf("\tALCT %s device %1i IDcode   = %8.8X\n",alct_chip_type[chip_id].c_str(),chip_id,idcode);
	}	// close for chip_id

// Read FPGA/PROM USERCodes (8 bit opcode)
	for (chip_id=0; chip_id<=1; ++chip_id) {
	if (chip_id==0) opcode = 0x08;					// FPGA USERcode opcode
	if (chip_id==1) opcode = 0xFD;					// PROM USERcode opcode
	reg_len=32;										// IDcode length
													// FPGA,PROM chip
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],idcode,32,0);
	printf("\tALCT %s device %1i USERcode = %8.8X\n",alct_chip_type[chip_id].c_str(),chip_id,idcode);
	}	// close for chip_id

// Create fat 0 for writing to data registers
	for (i=0; i<mxbitstream; ++i) {
	tdi[i]=0;
	}

// Select ALCT Mezzanine FPGA control JTAG chain from TMB boot register
	ichain = 0x0002;								// ALCT Mezzanine control jtag chain
	adr    = boot_adr;								// Boot register address
	vme_jtag_anystate_to_rti(adr,ichain);			// Take TAP to RTI

// Read ALCT ID register (5 bit opcode)
	chip_id = 0;
	opcode  = IDRead;								// ALCT ID register opcode
	reg_len = 40;									// Register length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	dprintf(stdout,"tdo="); for (i=0; i<reg_len; ++i) dprintf(stdout,"%1i",tdo[i]); dprintf(stdout,"\n");

// Decode ALCT ID register
	for (i=0; i<=39; ++i) {
	rsa[i]=tdo[i];
	}

	tdi_to_i4(&tdo[ 0], alct_idreg[0], 32,0);
	tdi_to_i4(&tdo[32], alct_idreg[1],  8,0);
	printf("\n\tALCT ID =%8.8X%8.8X\n",alct_idreg[1],alct_idreg[0]);

	tdi_to_i4(&rsa[ 0], rsa_chip_id,  4,0);
	tdi_to_i4(&rsa[ 4], rsa_version,  4,0);
	tdi_to_i4(&rsa[ 8], rsa_year,    16,0);
	tdi_to_i4(&rsa[24], rsa_day,      8,0);
	tdi_to_i4(&rsa[32], rsa_month,    8,0);

	printf("\trsa_chip_id %4.1X\n",rsa_chip_id);
	printf("\trsa_version %4.1X\n",rsa_version);
	printf("\trsa_year    %4.4X\n",rsa_year);
	printf("\trsa_day     %4.2X\n",rsa_day);
	printf("\trsa_month   %4.2X\n",rsa_month);

// Read ALCT digital serial numbers
// Reset DS2401
	vme_jtag_write_ir(adr,ichain,chip_id,SNreset );
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);

// Send read command 33h to ibutton chip
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);

	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite1);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);
	vme_jtag_write_ir(adr,ichain,chip_id,SNwrite0);

// Read 64 bits of DSN bit by bit
	reg_len    = 1;											// Register length
	alct_sn[0] = 0;
	alct_sn[1] = 0;

	for (i=0; i<=63; ++i) {
	vme_jtag_write_ir(adr,ichain,chip_id,SNread);
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	ibit=tdo[0];
	if (i>= 0 && i < 31) alct_sn[0] = alct_sn[0] | (ibit<<i);
	if (i>=32 && i < 63) alct_sn[1] = alct_sn[1] | (ibit<<i);
	}

	printf("\n\tALCT DSN =%8.8X%8.8X\n",alct_sn[1],alct_sn[0]);

	alct_dsn_mfg = (alct_sn[0] >>  0) & 0x00FF;
	alct_dsn     = (alct_sn[0] >>  8) & 0xFFFFFF;
	alct_dsn_crc = (alct_sn[0] >> 28) & 0x00FF;

	printf("\n\tDigital Serial for ALCT");
	printf(" CRC=%2.2X",alct_dsn_crc);
	printf(" DSN=%6.6X",alct_dsn);
	printf(" MFG=%2.2X",alct_dsn_mfg);
	printf("\n");

// Select ALCT Mezzanine FPGA control JTAG chain from TMB boot register
	ichain = 0x0002;							// ALCT Mezzanine control jtag chain
	adr    = boot_adr;							// Boot register address
	vme_jtag_anystate_to_rti(adr,ichain);		// Take TAP to RTI

// Read ALCT Configuration register (5 bit opcode)
	chip_id = 0;
	opcode  = RdCfg;							// ALCT cfg register opcode
//	opcode  = 0x06;								// ALCT cfg register opcode
	reg_len = 69;								// Register length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	dprintf(stdout,"tdo="); for (i=0; i<reg_len; ++i) dprintf(stdout,"%1i",tdo[i]); dprintf(stdout,"\n");

// Decode ALCT configuration register
	for (i=0; i<69; ++i) {
	rsa[i]=tdo[i];
	}

	tdi_to_i4(&tdo[ 0], alct_cfgreg[0], 32,0);
	tdi_to_i4(&tdo[32], alct_cfgreg[1], 32,0);
	tdi_to_i4(&tdo[64], alct_cfgreg[2],  5,0);

	printf("\n\tALCT Cfg=%2.2X%8.8X%8.8X\n",alct_cfgreg[2],alct_cfgreg[1],alct_cfgreg[0]);

	tdi_to_i4(&rsa[ 0], rsa_trig_mode,    2,0);
	tdi_to_i4(&rsa[ 2], rsa_ext_trig_en,  1,0);
	tdi_to_i4(&rsa[ 3], rsa_pretrig_halt, 1,0);
	tdi_to_i4(&rsa[ 4], rsa_inject,       1,0);
	tdi_to_i4(&rsa[ 5], rsa_inject_mode,  1,0);
	tdi_to_i4(&rsa[ 6], rsa_inject_mask,  7,0);
	tdi_to_i4(&rsa[13], rsa_nph_thresh,   3,0);
	tdi_to_i4(&rsa[16], rsa_nph_pattern,  3,0);
	tdi_to_i4(&rsa[19], rsa_drift_delay,  2,0);
	tdi_to_i4(&rsa[21], rsa_fifo_tbins,   5,0);
	tdi_to_i4(&rsa[26], rsa_fifo_pretrig, 5,0);
	tdi_to_i4(&rsa[31], rsa_fifo_mode,    2,0);
	tdi_to_i4(&rsa[33], rsa_fifo_lastlct, 3,0);
	tdi_to_i4(&rsa[36], rsa_l1a_delay,    8,0);
	tdi_to_i4(&rsa[44], rsa_l1a_window,   4,0);
	tdi_to_i4(&rsa[48], rsa_l1a_offset,   4,0);	
	tdi_to_i4(&rsa[52], rsa_l1a_internal, 1,0);
	tdi_to_i4(&rsa[53], rsa_board_id,     3,0);
	tdi_to_i4(&rsa[56], rsa_bxn_offset,   4,0);
	tdi_to_i4(&rsa[60], rsa_ccb_enable,   1,0);
	tdi_to_i4(&rsa[61], rsa_alct_jtag_ds, 1,0);
	tdi_to_i4(&rsa[62], rsa_alct_tmode,   2,0);
	tdi_to_i4(&rsa[64], rsa_alct_amode,   2,0);
	tdi_to_i4(&rsa[66], rsa_alct_maskall, 1,0);
	tdi_to_i4(&rsa[67], rsa_trig_info_en, 1,0);
	tdi_to_i4(&rsa[68], rsa_sn_select,    1,0);

	printf("\t 0 rsa_trig_mode    %3i\n",rsa_trig_mode);
	printf("\t 2 rsa_ext_trig_en  %3i\n",rsa_ext_trig_en);
	printf("\t 3 rsa_pretrig_halt %3i\n",rsa_pretrig_halt);
	printf("\t 4 rsa_inject       %3i\n",rsa_inject);
	printf("\t 5 rsa_inject_mode  %3i\n",rsa_inject_mode);
	printf("\t 6 rsa_inject_mask  %3i\n",rsa_inject_mask);
	printf("\t13 rsa_nph_thresh   %3i\n",rsa_nph_thresh);
	printf("\t16 rsa_nph_pattern  %3i\n",rsa_nph_pattern);
	printf("\t19 rsa_drift_delay  %3i\n",rsa_drift_delay);
	printf("\t21 rsa_fifo_tbins   %3i\n",rsa_fifo_tbins);
	printf("\t26 rsa_fifo_pretrig %3i\n",rsa_fifo_pretrig);
	printf("\t31 rsa_fifo_mode    %3i\n",rsa_fifo_mode);
	printf("\t33 rsa_fifo_lastlct %3i\n",rsa_fifo_lastlct);
	printf("\t36 rsa_l1a_delay    %3i\n",rsa_l1a_delay);
	printf("\t44 rsa_l1a_window   %3i\n",rsa_l1a_window);
	printf("\t48 rsa_l1a_offset   %3i\n",rsa_l1a_offset);
	printf("\t52 rsa_l1a_internal %3i\n",rsa_l1a_internal);
	printf("\t53 rsa_board_id     %3i\n",rsa_board_id);
	printf("\t56 rsa_bxn_offset   %3i\n",rsa_bxn_offset);
	printf("\t60 rsa_ccb_enable   %3i\n",rsa_ccb_enable);
	printf("\t61 rsa_alct_jtag_ds %3i\n",rsa_alct_jtag_ds);
	printf("\t62 rsa_alct_tmode   %3i\n",rsa_alct_tmode);
	printf("\t64 rsa_alct_amode   %3i\n",rsa_alct_amode);
	printf("\t66 rsa_alct_maskall %3i\n",rsa_alct_maskall);
	printf("\t67 rsa_trig_info_en %3i\n",rsa_trig_info_en);
	printf("\t68 rsa_sn_select    %3i\n",rsa_sn_select);
	printf("\n");

	printf("\n\tWrite new data? bit,len,val <cr=no> ");
	gets(line);
	if (line[0]==NULL) goto L2300;
	sscanf(line,"%i %i %X",&ibit,&ilen,&ival);	

// Set new ALCT cfg bits
	bit_to_array(ival,ivalarray,ilen);

	for (i=0; i<=68; ++i) {
	rsa[i]=tdo[i+1];
	if (i>=ibit && i<=(ibit+ilen-1)) rsa[i]=ivalarray[i-ibit];
	}

// Write ALCT Configuration register (5 bit opcode)
	chip_id = 0;
	opcode  = WrCfg;										// ALCT cfg register opcode
	reg_len = 69;											// Register length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,rsa,tdo,reg_len);	// Write data
	dprintf(stdout,"tdo="); for (i=0; i<reg_len; ++i) dprintf(stdout,"%1i",tdo[i]); dprintf(stdout,"\n");

	goto L2300;

//------------------------------------------------------------------------------
//	ALCT JTAG read/write: Debug firmware
//------------------------------------------------------------------------------
L23200:

	printf("\tMake sure you removed the JTAG cable!\n");

//	Chain ID	Section		 Control or Program
//	--------	------------ ------------------
//	  0			Slow Control control registers
//	  1			Slow Control PROM
//	  2			Mezzanine    control registers (alct normal firmware)
//	  3			Mezzanine    FPGA+PROM
//
//
//	ALCT Debug Firmware BSCAN Register USER1 readonly:
//	Field		Len	Typical	Description
//	-------		---	-------	--------------------------
//	[ 3: 0]		4	B		Begin marker
//	[ 7: 4]		4	A		Version ID
//	[23: 8]		16	0823	Version date
//	[39:24]		16	2004	Version date
//	[40]		1	1		Mez FPGA reports done
//	[41]		1	1		Slow control FPGA reports done
//	[42]		1	1		DLL locked
//	[43]		1	1		Clock enable
//	[47:44]		4	C		USER2 alignment marker
//	[48]		1	0		Cmd_sync_mode 1=sync mode	
//	[49]		1	-		1=80MHz synch mode
//	[50]		1	-		First  80MHz phase data ok
//	[51]		1	-		Second 80MHz phase data ok
//	[63:52]		12	-		First  80MHz phase data alct_rx_1st[16:5]
//	[75:64]		12			Second 80MHz phase data alct_rx_2nd[16:5]
//	[76]		1	1		cmd_l1a_en, enable l1a readout on ext_trig
//	[77]		1	1		cmd_trig_en,enable trigger word on ext_trig
//	[78]		1	0		cmd_dummy
//	[79:76]		4	00		Free
//	[83:80]		4=	E		End marker
//
//
//  ALCT Debug Firmware BSCAN Register USER2 write/read:
//	Field		Len	Typical	Description
//	-------		---	-------	--------------------------
//	[ 3: 0]		4	C		Alignment marker
//	[4]			1	0		1=sync_mode
//	[5]			1	1		cmd_l1a_en, enable l1a readout on ext_trig
//	[6]			1	1		cmd_trig_en,enable trigger word on ext_trig
//	[7]			1	0		cmd_dummy
//	[23:8]		16	FFFF	tx_en0, enable alct0 trigger bits
//	[39:24]		16	FFFF	tx_en1, enable alct1 trigger bits
//
//------------------------------------------------------------------------------
// Select ALCT Mezzanine FPGA programming JTAG chain from TMB boot register
	ichain = 0x0003;							// ALCT Mezzanine pgm jtag chain
	adr    = boot_adr;							// Boot register address
	vme_jtag_anystate_to_rti(adr,ichain);		// Take TAP to RTI

// Read Virtex-E FPGA (5-bit opcode) and XC18V04 PROM IDcodes (8-bit opcode)
	for (chip_id=0; chip_id<=1; ++chip_id) {
	if (chip_id==0) opcode = 0x09;				// FPGA IDcode opcode, expect v0A30093
	if (chip_id==1) opcode = 0xFE;				// PROM IDcode opcode
	reg_len=32;									// IDcode length
												// FPGA,PROM chip
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],idcode,32,0);
	printf("\tALCT %s device %1i IDcode   = %8.8X\n",alct_chip_type[chip_id].c_str(),chip_id,idcode);
	}

// Read FPGA/PROM USERCodes (8 bit opcode)
	for (chip_id=0; chip_id<=1; ++chip_id) {
	if (chip_id==0) opcode = 0x08;				// FPGA USERcode opcode
	if (chip_id==1) opcode = 0xFD;				// PROM USERcode opcode
	reg_len=32;									// IDcode length
												// FPGA,PROM chip
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

	tdi_to_i4(&tdo[0],idcode,32,0);
	printf("\tALCT %s device %1i USERcode = %8.8X\n",alct_chip_type[chip_id].c_str(),chip_id,idcode);
	}

// Create fat 0 for writing to data registers
	for (i=0; i<mxbitstream; ++i) {
	tdi[i]=0;
	}

// Select ALCT Mezzanine FPGA VirtexE JTAG chain from TMB boot register
	ichain = 0x0003;							// ALCT VirtexE
	adr    = boot_adr;							// Boot register address
	vme_jtag_anystate_to_rti(adr,ichain);		// Take TAP to RTI

// Read ALCT VirtexE USER1 register (5 bit opcode)
	chip_id = 0;
	opcode  = 0x02;								// VirtexE USER1 opcode
	reg_len = 84;								// Register length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	dprintf(stdout,"tdo="); for (i=0; i<reg_len; ++i) dprintf(stdout,"%1i",tdo[i]); dprintf(stdout,"\n");
	if (ifunc<0) goto L23200;

// Decode ALCT USER1 register
	for (i=0; i<reg_len; ++i) {
	rsd[i]=tdo[i];
	}

	tdi_to_i4(&tdo[ 0], alct_user1[0], 32,0);
	tdi_to_i4(&tdo[32], alct_user1[1], 32,0);
	tdi_to_i4(&tdo[64], alct_user1[2], 16,0);

	printf("\n\tALCT USER1 = %8.8X%8.8X%8.8X\n",alct_user1[2],alct_user1[1],alct_user1[0]);

	tdi_to_i4(&rsd[ 0], rsd_begin,          4,0);
	tdi_to_i4(&rsd[ 4], rsd_version,        4,0);
	tdi_to_i4(&rsd[ 8], rsd_monthday,      16,0);
	tdi_to_i4(&rsd[24], rsd_year,          16,0);
	tdi_to_i4(&rsd[40], rsd_mc_done,        1,0);
	tdi_to_i4(&rsd[41], rsd_sc_done,        1,0);
	tdi_to_i4(&rsd[42], rsd_clock_lock,     1,0);
	tdi_to_i4(&rsd[43], rsd_clock_en,       1,0);
	tdi_to_i4(&rsd[44], rsd_cmd_align,      4,0);
	tdi_to_i4(&rsd[48], rsd_cmd_sync_mode,  1,0);
	tdi_to_i4(&rsd[49], rsd_sync_mode,      1,0);
	tdi_to_i4(&rsd[50], rsd_sync_rx_1st_ok, 1,0);
	tdi_to_i4(&rsd[51], rsd_sync_rx_2nd_ok, 1,0);
	tdi_to_i4(&rsd[52], rsd_alct_rx_1st,   12,0);
	tdi_to_i4(&rsd[64], rsd_alct_rx_2nd,   12,0);
	tdi_to_i4(&rsd[76], rsd_cmd_l1a_en,     1,0);
	tdi_to_i4(&rsd[77], rsd_cmd_trig_en,    1,0);
	tdi_to_i4(&rsd[78], rsd_cmd_dummy,      1,0);
	tdi_to_i4(&rsd[79], rsd_free0,          1,0);
	tdi_to_i4(&rsd[80], rsd_end,            4,0);

	printf("\trsd_begin          %4.1X\n",rsd_begin);
	printf("\trsd_version        %4.1X\n",rsd_version);
	printf("\trsd_monthday       %4.4X\n",rsd_monthday);
	printf("\trsd_year           %4.4X\n",rsd_year);
	printf("\trsd_mc_done        %4.1X\n",rsd_mc_done);
	printf("\trsd_sc_done        %4.1X\n",rsd_sc_done);
	printf("\trsd_clock_lock     %4.1X\n",rsd_clock_lock);
	printf("\trsd_clock_en       %4.1X\n",rsd_clock_en);
	printf("\trsd_cmd_align      %4.1X\n",rsd_cmd_align);
	printf("\trsd_cmd_sync_mode  %4.1X\n",rsd_cmd_sync_mode);
	printf("\trsd_sync_mode      %4.1X\n",rsd_sync_mode);
	printf("\trsd_sync_rx_1st_ok %4.1X\n",rsd_sync_rx_1st_ok);
	printf("\trsd_sync_rx_2nd_ok %4.1X\n",rsd_sync_rx_2nd_ok);
	printf("\trsd_alct_rx_1st    %4.3X\n",rsd_alct_rx_1st);
	printf("\trsd_alct_rx_2nd    %4.3X\n",rsd_alct_rx_2nd);
	printf("\trsd_cmd_l1a_en     %4.3X\n",rsd_cmd_l1a_en);
	printf("\trsd_cmd_trig_en    %4.3X\n",rsd_cmd_trig_en);
	printf("\trsd_cmd_dummy      %4.3X\n",rsd_cmd_dummy);
	printf("\trsd_free0          %4.1X\n",rsd_free0);
	printf("\trsd_end            %4.1X\n",rsd_end);

// Read ALCT VirtexE USER2 register (5 bit opcode)
	chip_id = 0;
	opcode  = 0x03;								// VirtexE USER2 opcode
	reg_len = 40;								// Register length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	dprintf(stdout,"tdo="); for (i=0; i<reg_len; ++i) dprintf(stdout,"%1i",tdo[i]); dprintf(stdout,"\n");

// Decode ALCT USER2 register
	for (i=0; i<reg_len; ++i) {
	rsd[i]=tdo[i];
	}

	tdi_to_i4(&tdo[ 0],alct_user2[0],32,0);
	tdi_to_i4(&tdo[32],alct_user2[1], 8,0);

	printf("\tALCT USER2 = %8.8X%8.8X\n",alct_user2[1],alct_user2[0]);

	tdi_to_i4(&rsd[ 0], rsd_cmd_align,     4,0);
	tdi_to_i4(&rsd[ 4], rsd_cmd_sync_mode, 1,0);
	tdi_to_i4(&rsd[ 5], rsd_cmd_l1a_en,    1,0);
	tdi_to_i4(&rsd[ 6], rsd_cmd_trig_en,   1,0);
	tdi_to_i4(&rsd[ 7], rsd_cmd_dummy,     1,0);
	tdi_to_i4(&rsd[ 8], rsd_tx_en0,       16,0);
	tdi_to_i4(&rsd[24], rsd_tx_en1,       16,0);

	printf("\trsd[3:0]   rsd_cmd_align     %4.1X\n",rsd_cmd_align);
	printf("\trsd[4]     rsd_cmd_sync_mode %4.1X\n",rsd_cmd_sync_mode);
	printf("\trsd[5]     rsd_cmd_l1a_en    %4.1X\n",rsd_cmd_l1a_en);
	printf("\trsd[6]     rsd_cmd_trig_en   %4.1X\n",rsd_cmd_trig_en);
	printf("\trsd[7]     rsd_cmd_dummy     %4.1X\n",rsd_cmd_dummy);
	printf("\trsd[23:8]  rsd_tx_en0        %4.4X\n",rsd_tx_en0);
	printf("\trsd[39:24] rsd_tx_en1        %4.4X\n",rsd_tx_en1);

// Restore USER2 because readout was destructive, alas
	for (i=0; i<reg_len; ++i) {
	tdi[i]=tdo[i];
	}

	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode

// Write new data to USER2
	printf("\n\tWrite new data? bit,len,val <cr=no> ");
	gets(line);
	if (line[0]==NULL) goto L23299;
	sscanf(line,"%i %i %X",&ibit,&ilen,&ival);	

// Set new ALCT USER2 bits
	bit_to_array(ival,ivalarray,ilen);

	for (i=0; i<reg_len; ++i) {
	if (i>=ibit && i<=(ibit+ilen-1)) {
	rsd[i]=ivalarray[i-ibit];
	}
	}

// Write ALCT USER2 register (5 bit opcode)
	chip_id = 0;
	opcode  = 0x03;								// VirtexE USER2 opcode
	reg_len = 40;								// Register length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,rsd,tdo,reg_len);	// Write data
	dprintf(stdout,"tdo="); for (i=0; i<reg_len; ++i) dprintf(stdout,"%1i",tdo[i]); dprintf(stdout,"\n");

L23299:
	goto L2300;

//------------------------------------------------------------------------------
//	ALCT JTAG read/write: Debug firmware
//------------------------------------------------------------------------------
L23800:
	printf("\tMake sure you removed the JTAG cable!\n");
	printf("\tinfinite loop anystate to rti on chain 3\n");
	
L23810:
	ichain = 0x0002;							// ALCT User
	adr    = boot_adr;							// Boot register address
	vme_jtag_anystate_to_rti(adr,ichain);		// Take TAP to RTI
	sleep(1);

// Read Virtex-E FPGA (5-bit opcode) and XC18V04 PROM IDcodes (8-bit opcode)
	chip_id = 0;
	opcode  = 0x09;								// FPGA IDcode opcode, expect v0A30093
	reg_len = 32;								// IDcode length
	vme_jtag_write_ir(adr,ichain,chip_id,opcode);			// Set opcode
	vme_jtag_write_dr(adr,ichain,chip_id,tdi,tdo,reg_len);	// Write 0's read idcode
	sleep(1);

	if (ifunc>0) goto L23810;
	goto L2300;
}

//------------------------------------------------------------------------------
//
// Service routines for main
//
//------------------------------------------------------------------------------
//	Check data read vs data expected
//------------------------------------------------------------------------------
	void ck(string msg_string, int data_read, int data_expect)
{	
	if (data_read != data_expect) {
	fprintf(stdout,  "ERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
	fprintf(log_file,"ERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
//	pause(" ");
	}
	return;
}
//------------------------------------------------------------------------------
//	Check data read vs data expected, with status return
//------------------------------------------------------------------------------
	int cks(string msg_string, int data_read, int data_expect)
{
	int status;

	status = 0;	// good return
	if (data_read != data_expect) {
	status = 1;	// bad return
	fprintf(stdout,  "\tERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
	fprintf(log_file,"\tERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
//	pause(" ");
	}

	return status;
}
//------------------------------------------------------------------------------
//	Check data read vs data expected, floating point version  with tolerance
//------------------------------------------------------------------------------
	void tok(string msg_string, double fdata_read, double fdata_expect, double tolerance, int status)
{
	double err;
	double errpct;

	err    = (fdata_read-fdata_expect)/__max(fdata_expect,.01);
	errpct = err*100.;

	status=0;
	if (abs(err)>tolerance) {
	status=1;
	fprintf(stdout,  "\tERRm: in %s: read=%10.4g expect=%10.4g %10.2f\n",msg_string.c_str(),fdata_read,fdata_expect,errpct);
	fprintf(log_file,"\tERRm: in %s: read=%10.4g expect=%10.4g %10.2f\n",msg_string.c_str(),fdata_read,fdata_expect,errpct);
	}

	return;
}
//------------------------------------------------------------------------------
//	Inquire prompt for integer
//------------------------------------------------------------------------------
	void inquire(string prompt, const int &minv, const int &maxv, const int &radix, int &now)
{
	char	line[80];
	int		i;
	int		n;

ask:
	printf(prompt.c_str(),now);
	gets(line);
	n=strlen(line);
	if (radix==16)	sscanf(line,"%x",&i);
	else			sscanf(line,"%i",&i);

	if ((n!=0) && ((i<minv) || (i>maxv))) {
	if (radix==16) printf("Out of range. Expect %X to %X\n",minv,maxv);
	else           printf("Out of range. Expect %i to %i\n",minv,maxv);
	goto ask;
	}
	if (n!=0) now = i;
	return;
}
//------------------------------------------------------------------------------
//	Inquire prompt for two integers
//------------------------------------------------------------------------------
	void inquir2(string prompt, const int &min, const int &max, const int &radix, int &num, int &now)
{
	char	line[80];
	int		i;
	int		n;

ask:
	printf(prompt.c_str(),num,now);
	gets(line);
	n=strlen(line);
	if (radix==16)	sscanf(line,"%x",&i);
	else			sscanf(line,"%i",&i);

	if ((n!=0) && ((i<min) || (i>max))) {
	if (radix==16) printf("Out of range. Expect %X to %X\n",min,max);
	else           printf("Out of range. Expect %i to %i\n",min,max);
	goto ask;
	}
	if (n!=0) now = i;
	return;
}
//------------------------------------------------------------------------------
//	Inquire prompt for long integer
//------------------------------------------------------------------------------
	void inquirl(string prompt, const int &min, const int &max, const int &radix, long int &now)
{
	char		line[80];
	long int	i;
	int			n;

ask:
	printf(prompt.c_str(),now);
	gets(line);
	n=strlen(line);
	if (radix==16)	sscanf(line,"%x",&i);
	else			sscanf(line,"%i",&i);

	if ((n!=0) && ((i<min) || (i>max))) {
	if (radix==16) printf("Out of range. Expect %X to %X\n",min,max);
	else           printf("Out of range. Expect %i to %i\n",min,max);
	goto ask;
	}
	if (n!=0) now = i;
	return;
}
//------------------------------------------------------------------------------
//	Inquire prompt for bool
//------------------------------------------------------------------------------
	void inquirb(string prompt, bool &now)
{
	char	line[80];
	char	i;
	int		n;

ask:
	printf(prompt.c_str(),yesno(now&0x1));
	gets(line);
	n=strlen(line);
	sscanf(line,"%c",&i);

	if (n!=0)
	{
	if (i!='y' && i!='Y' && i!='n' && i!='N') {printf("\tEnter y or n\n"); goto ask;} 
	if (i=='y' || i=='Y') now = true;
	if (i=='n' || i=='N') now = false;
	}
	return;
}
//------------------------------------------------------------------------------
//	Pass Fail prompt
//------------------------------------------------------------------------------
	bool pass_fail(string prompt)
{
	char	line[80];
	int		i;
	int		n;
	bool	ans;

ask:
	printf(prompt.c_str());
	gets(line);
	n = strlen(line);
	i = line[0];

	if (n!=0 && (i=='p' || i=='P')) ans = true; 
	else
	if (n!=0 && (i=='f' || i=='F')) ans = false;
	else {
	printf("Enter P or F\n");
	goto ask;
	}
	return ans;
}
//------------------------------------------------------------------------------
//	Display Test OK
//------------------------------------------------------------------------------
	void aok(string msg_string)
{	
	int		tab           = 45;
	int		msg_spaces    = msg_string.length();
	int		insert_spaces = tab-msg_spaces;
	string	spaces        = " ";

	for (int i=0; i<=insert_spaces; ++i) spaces.append(string(" "));

	fprintf(log_file,"\t%s %s OK\n",msg_string.c_str(),spaces.c_str());	// log file
	fprintf(stdout,  "\t%s %s OK\n",msg_string.c_str(),spaces.c_str());	// screen
	return;
}
//------------------------------------------------------------------------------
//	Display Test OK or FAIL
//------------------------------------------------------------------------------
	void aokf(string msg_string, const int itest, const int status)	
{
	string	sstat[2]={"FAIL","PASS"};

	int		tab           = 45;
	int		msg_spaces    = msg_string.length();
	int		insert_spaces = tab-msg_spaces;
	string	spaces        = " ";

	for (int i=0; i<=insert_spaces; ++i) spaces.append(string(" "));

	fprintf(log_file,"\t%3i %s %s %s\n",itest,msg_string.c_str(),spaces.c_str(),sstat[status].c_str());	// log file
	fprintf(stdout,  "\t%3i %s %s %s\n",itest,msg_string.c_str(),spaces.c_str(),sstat[status].c_str());	// screen

	return;
}
//------------------------------------------------------------------------------
//	Convert integer bit string to an array
//------------------------------------------------------------------------------
	void bit_to_array(const int &idata, int iarray[], const int &n)
	{
	int i;

	for (i=0; i<n; ++i) {
	iarray[i]=(idata >> i) & 0x00000001;
	}
	return;
}
//--------------------------------------------------------------------------------
//	Wait for specified number of milliseconds, probably MS Visual Studio specific
//--------------------------------------------------------------------------------
	void sleep(clock_t msec)
{
   clock_t goal;
   goal = msec + clock();
   while (goal > clock());
}
//--------------------------------------------------------------------------------
//	Count 0s in a bit array
//--------------------------------------------------------------------------------
	int count0s(char tdo[], int &nbits)
	{
	int zeros;
	int i;
	zeros=0;

	for (i=0; i<nbits; ++i) if (tdo[i]==0) zeros++;

	return zeros;
}
//--------------------------------------------------------------------------------
//	Count 1s in a bit array
//--------------------------------------------------------------------------------
	int count1s(char tdo[], int &nbits)
{
	int ones;
	int i;
	ones=0;

	for (i=0; i<nbits; ++i) if (tdo[i]==1) ones++;

	return ones;
}
//------------------------------------------------------------------------------
// The bitter end
//------------------------------------------------------------------------------
