//------------------------------------------------------------------------------
//  ALCT ASIC test using JTAG
//------------------------------------------------------------------------------
//  11/29/11 Initial
//  12/01/11 Change bank select bits to ccb_brcst
//------------------------------------------------------------------------------
//  Headers
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
//  Common
//------------------------------------------------------------------------------
    FILE            *log_file;
    FILE            *jtag_file;
    bool            jtaglogmode=false;

// Common/jtag_common/
    const int       mxbitstream = 1000;

    char            tms[mxbitstream];
    char            tdi[mxbitstream];
    char            tdo[mxbitstream];

    int             T[6][4];
    int             C[6][16];

    int             nframes;
    int             step_mode;
    int             ichip;
    int             ibit;
    int             ichan;
    int             tdo_bit0;
    int             ibank=0;
    int             ichain=2;
    int             idelay=15;
    int             idelay_bit[4];
    long int        i4;

    unsigned short  ccb_cmd_default;
    unsigned short  ccb_cmd_wr;

    int             ibank_bit[2];
    int             ccb_brcst_bit[8];
    int             ccb_brcst;
    int             ccb_strobe;
    int             nrs_dly;
    int             seltst_dly;

    string          log_file_name;

//------------------------------------------------------------------------------
//  Common/TMB_VME_addresses
//------------------------------------------------------------------------------
    unsigned long       base_adr;
    const unsigned long tmb_global_slot         = 26;
    const unsigned long tmb_brcst_slot          = 27;
    const unsigned long tmb_boot_adr            = 0x070000;
//------------------------------------------------------------------------------
    const unsigned long vme_idreg_adr           = 0x000000; // For tmb2005 and bdtest_v5
    const unsigned long vme_status_adr          = 0x000008;
    const unsigned long vme_adr0_adr            = 0x00000A;
    const unsigned long vme_adr1_adr            = 0x00000C;
    const unsigned long vme_loopbk_adr          = 0x00000E;

    const unsigned long vme_usr_jtag_adr        = 0x000010;
    const unsigned long vme_prom_adr            = 0x000012;

    const unsigned long vme_dddsm_adr           = 0x000014;
    const unsigned long vme_ddd0_adr            = 0x000016;
    const unsigned long vme_ddd1_adr            = 0x000018;
    const unsigned long vme_ddd2_adr            = 0x00001A;
    const unsigned long vme_dddoe_adr           = 0x00001C;
    const unsigned long vme_ratctrl_adr         = 0x00001E;

    const unsigned long vme_step_adr            = 0x000020;
    const unsigned long vme_led_adr             = 0x000022;
    const unsigned long vme_adc_adr             = 0x000024;
    const unsigned long vme_dsn_adr             = 0x000026;
    //------------------------------------------------------------------------------
    const unsigned long mod_cfg_adr             = 0x000028; // For tmb2005 normal firmware
    const unsigned long ccb_cfg_adr             = 0x00002A;
    const unsigned long ccb_trig_adr            = 0x00002C;
    const unsigned long ccb_stat0_adr           = 0x00002E;
    const unsigned long alct_cfg_adr            = 0x000030;
    const unsigned long alct_inj_adr            = 0x000032;
    const unsigned long alct0_inj_adr           = 0x000034;
    const unsigned long alct1_inj_adr           = 0x000036;
    const unsigned long alct_stat_adr           = 0x000038;
    const unsigned long alct_alct0_adr          = 0x00003A;
    const unsigned long alct_alct1_adr          = 0x00003C;
    const unsigned long alct_fifo_adr           = 0x00003E;
    const unsigned long dmb_mon_adr             = 0x000040;
    const unsigned long cfeb_inj_adr            = 0x000042;
    const unsigned long cfeb_inj_adr_adr        = 0x000044;
    const unsigned long cfeb_inj_wdata_adr      = 0x000046;
    const unsigned long cfeb_inj_rdata_adr      = 0x000048;
    const unsigned long hcm001_adr              = 0x00004A;
    const unsigned long hcm023_adr              = 0x00004C;
    const unsigned long hcm045_adr              = 0x00004E;
    const unsigned long hcm101_adr              = 0x000050;
    const unsigned long hcm123_adr              = 0x000052;
    const unsigned long hcm145_adr              = 0x000054;
    const unsigned long hcm201_adr              = 0x000056;
    const unsigned long hcm223_adr              = 0x000058;
    const unsigned long hcm245_adr              = 0x00005A;
    const unsigned long hcm301_adr              = 0x00005C;
    const unsigned long hcm323_adr              = 0x00005E;
    const unsigned long hcm345_adr              = 0x000060;
    const unsigned long hcm401_adr              = 0x000062;
    const unsigned long hcm423_adr              = 0x000064;
    const unsigned long hcm445_adr              = 0x000066;
    const unsigned long seq_trig_en_adr         = 0x000068;
    const unsigned long seq_trig_dly0_adr       = 0x00006A;
    const unsigned long seq_trig_dly1_adr       = 0x00006C;
    const unsigned long seq_id_adr              = 0x00006E;
    const unsigned long seq_clct_adr            = 0x000070;
    const unsigned long seq_fifo_adr            = 0x000072;
    const unsigned long seq_l1a_adr             = 0x000074;
    const unsigned long seq_offset0_adr         = 0x000076;
    const unsigned long seq_clct0_adr           = 0x000078;
    const unsigned long seq_clct1_adr           = 0x00007A;
    const unsigned long seq_trig_src_adr        = 0x00007C;
    const unsigned long dmb_ram_adr             = 0x00007E;
    const unsigned long dmb_wdata_adr           = 0x000080;
    const unsigned long dmb_wdcnt_adr           = 0x000082;
    const unsigned long dmb_rdata_adr           = 0x000084;
    const unsigned long tmb_trig_adr            = 0x000086;
    const unsigned long mpc0_frame0_adr         = 0x000088;
    const unsigned long mpc0_frame1_adr         = 0x00008A;
    const unsigned long mpc1_frame0_adr         = 0x00008C;
    const unsigned long mpc1_frame1_adr         = 0x00008E;
    const unsigned long mpc_inj_adr             = 0x000090;
    const unsigned long mpc_ram_adr             = 0x000092;
    const unsigned long mpc_ram_wdata_adr       = 0x000094;
    const unsigned long mpc_ram_rdata_adr       = 0x000096;
    unsigned long       scp_ctrl_adr            = 0x000098;
    unsigned long       scp_rdata_adr           = 0x00009A;
    const unsigned long ccb_cmd_adr             = 0x00009C;

    const unsigned long buf_stat0_adr           = 0x00009E;
    const unsigned long buf_stat1_adr           = 0x0000A0;
    const unsigned long buf_stat2_adr           = 0x0000A2;
    const unsigned long buf_stat3_adr           = 0x0000A4;
    const unsigned long buf_stat4_adr           = 0x0000A6;
    const unsigned long alctfifo1_adr           = 0x0000A8;
    const unsigned long alctfifo2_adr           = 0x0000AA;
    const unsigned long seqmod_adr              = 0x0000AC;
    const unsigned long seqsm_adr               = 0x0000AE;
    const unsigned long seq_clctm_adr           = 0x0000B0;
    const unsigned long tmbtim_adr              = 0x0000B2;
    const unsigned long lhc_cycle_adr           = 0x0000B4;
    const unsigned long rpc_cfg_adr             = 0x0000B6;
    const unsigned long rpc_rdata_adr           = 0x0000B8;
    const unsigned long rpc_raw_delay_adr       = 0x0000BA;
    const unsigned long rpc_inj_adr             = 0x0000BC;
    const unsigned long rpc_inj_adr_adr         = 0x0000BE;
    const unsigned long rpc_inj_wdata_adr       = 0x0000C0;
    const unsigned long rpc_inj_rdata_adr       = 0x0000C2;
    const unsigned long rpc_tbins_adr           = 0x0000C4;
    const unsigned long rpc_rpc0_hcm_adr        = 0x0000C6;
    const unsigned long rpc_rpc1_hcm_adr        = 0x0000C8;
    const unsigned long bx0_delay_adr           = 0x0000CA;
    const unsigned long non_trig_adr            = 0x0000CC;
    const unsigned long scp_trig_adr            = 0x0000CE;
    const unsigned long cnt_ctrl_adr            = 0x0000D0;
    const unsigned long cnt_rdata_adr           = 0x0000D2;

    const unsigned long jtagsm0_adr             = 0x0000D4;
    const unsigned long jtagsm1_adr             = 0x0000D6;
    const unsigned long jtagsm2_adr             = 0x0000D8;

    const unsigned long vmesm0_adr              = 0x0000DA;
    const unsigned long vmesm1_adr              = 0x0000DC;
    const unsigned long vmesm2_adr              = 0x0000DE;
    const unsigned long vmesm3_adr              = 0x0000E0;
    const unsigned long vmesm4_adr              = 0x0000E2;

    const unsigned long dddrsm_adr              = 0x0000E4;
    const unsigned long dddr0_adr               = 0x0000E6;

    const unsigned long uptimer_adr             = 0x0000E8;
    const unsigned long bdstatus_adr            = 0x0000EA;

    const unsigned long bxn_clct_adr            = 0x0000EC;
    const unsigned long bxn_alct_adr            = 0x0000EE;

    const unsigned long layer_trig_adr          = 0x0000F0;
    const unsigned long ise_version_adr         = 0x0000F2;

    const unsigned long temp0_adr               = 0x0000F4;
    const unsigned long temp1_adr               = 0x0000F6;
    const unsigned long temp2_adr               = 0x0000F8;

    const unsigned long parity_adr              = 0x0000FA;
    const unsigned long ccb_stat1_adr           = 0x0000FC;
    const unsigned long bxn_l1a_adr             = 0x0000FE;
    const unsigned long l1a_lookback_adr        = 0x000100;
    const unsigned long seq_debug_adr           = 0x000102;

    const unsigned long alct_sync_ctrl_adr      = 0x000104; // ALCT sync mode control
    const unsigned long alct_sync_txdata_1st    = 0x000106; // ALCT sync mode transmit data, 1st in time
    const unsigned long alct_sync_txdata_2nd    = 0x000108; // ALCT sync mode transmit data, 2nd in time

    const unsigned long seq_offset1_adr         = 0x00010A;
    const unsigned long miniscope_adr           = 0x00010C;

    const unsigned long phaser0_adr             = 0x00010E;
    const unsigned long phaser1_adr             = 0x000110;
    const unsigned long phaser2_adr             = 0x000112;
    const unsigned long phaser3_adr             = 0x000114;
    const unsigned long phaser4_adr             = 0x000116;
    const unsigned long phaser5_adr             = 0x000118;
    const unsigned long phaser6_adr             = 0x00011A;

    const unsigned long delay0_int_adr          = 0x00011C;
    const unsigned long delay1_int_adr          = 0x00011E;

    const unsigned long sync_err_ctrl_adr       = 0x000120; // Synchronization Error Control

    const unsigned long cfeb_badbits_ctrl_adr   = 0x000122; // CFEB  Bad Bit Control/Status
    const unsigned long cfeb_badbits_timer_adr  = 0x000124; // CFEB  Bad Bit Check Interval

    const unsigned long cfeb0_badbits_ly01_adr  = 0x000126; // CFEB0 Bad Bit Array
    const unsigned long cfeb0_badbits_ly23_adr  = 0x000128; // CFEB0 Bad Bit Array
    const unsigned long cfeb0_badbits_ly45_adr  = 0x00012A; // CFEB0 Bad Bit Array

    const unsigned long cfeb1_badbits_ly01_adr  = 0x00012C; // CFEB1 Bad Bit Array
    const unsigned long cfeb1_badbits_ly23_adr  = 0x00012E; // CFEB1 Bad Bit Array
    const unsigned long cfeb1_badbits_ly45_adr  = 0x000130; // CFEB1 Bad Bit Array

    const unsigned long cfeb2_badbits_ly01_adr  = 0x000132; // CFEB2 Bad Bit Array
    const unsigned long cfeb2_badbits_ly23_adr  = 0x000134; // CFEB2 Bad Bit Array
    const unsigned long cfeb2_badbits_ly45_adr  = 0x000136; // CFEB2 Bad Bit Array

    const unsigned long cfeb3_badbits_ly01_adr  = 0x000138; // CFEB3 Bad Bit Array
    const unsigned long cfeb3_badbits_ly23_adr  = 0x00013A; // CFEB3 Bad Bit Array
    const unsigned long cfeb3_badbits_ly45_adr  = 0x00013C; // CFEB3 Bad Bit Array

    const unsigned long cfeb4_badbits_ly01_adr  = 0x00013E; // CFEB4 Bad Bit Array
    const unsigned long cfeb4_badbits_ly23_adr  = 0x000140; // CFEB4 Bad Bit Array
    const unsigned long cfeb4_badbits_ly45_adr  = 0x000142; // CFEB4 Bad Bit Array

    const unsigned long last_vme_adr            = 0x00011E; // Last valid address instantiated
    //------------------------------------------------------------------------------
    const unsigned long vme_gpio_adr            = 0x000028; // For bdtestv3
    const unsigned long vme_cfg_adr             = 0x00002A;

    const unsigned long cfeb0a_adr              = 0x00002C; // Repeats 2C-48 for CFEBs1-4
    const unsigned long cfeb0b_adr              = 0x00002E;
    const unsigned long cfeb0c_adr              = 0x000030;
    const unsigned long cfeb_offset_adr         = 0x000006;

    const unsigned long alct_rxa_adr            = 0x00004A;
    const unsigned long alct_rxb_adr            = 0x00004C;
    const unsigned long alct_rxc_adr            = 0x00004E;
    const unsigned long alct_rxd_adr            = 0x000050;

    const unsigned long dmb_rxa_adr             = 0x000052;
    const unsigned long dmb_rxb_adr             = 0x000054;
    const unsigned long dmb_rxc_adr             = 0x000056;
    const unsigned long dmb_rxd_adr             = 0x000058;

    const unsigned long mpc_rxa_adr             = 0x00005A;
    const unsigned long mpc_rxb_adr             = 0x00005C;

    const unsigned long rpc_rxa_adr             = 0x00005E;
    const unsigned long rpc_rxb_adr             = 0x000060;
    const unsigned long rpc_rxc_adr             = 0x000062;
    const unsigned long rpc_rxd_adr             = 0x000064;
    const unsigned long rpc_rxe_adr             = 0x000066;
    const unsigned long rpc_rxf_adr             = 0x000068;

    const unsigned long ccb_rxa_adr             = 0x00006A;
    const unsigned long ccb_rxb_adr             = 0x00006C;
    const unsigned long ccb_rxc_adr             = 0x00006E;
    const unsigned long ccb_rxd_adr             = 0x000070;

    const unsigned long alct_txa_adr            = 0x000072;
    const unsigned long alct_txb_adr            = 0x000074;

    const unsigned long rpc_txa_adr             = 0x000076;
    const unsigned long rpc_txb_adr             = 0x000078;

    const unsigned long dmb_txa_adr             = 0x00007A;
    const unsigned long dmb_txb_adr             = 0x00007C;
    const unsigned long dmb_txc_adr             = 0x00007E;
    const unsigned long dmb_txd_adr             = 0x000080;

    const unsigned long mpc_txa_adr             = 0x000082;
    const unsigned long mpc_txb_adr             = 0x000084;

    const unsigned long ccb_txa_adr             = 0x000086;
    const unsigned long ccb_txb_adr             = 0x000088;

    const unsigned long heater_adr              = 0x00008A; // Last bdtestv3 address instantiated

//------------------------------------------------------------------------------
//  Prototypes
//------------------------------------------------------------------------------
    #define         logical(L)      ((L)?'T':'F')
    #define         yesno(L)        ((L)?'y':'n')
    void            pause           (string s);
    void            stop            (string s);
    void            sleep           (clock_t msec);
    bool            pass_fail       (string prompt);

    void            aok             (string msg_string);
    void            aokf            (string msg_string, const int itest, const int status);
    void            ck              (string data_string, int data_read, int data_expect);
    int             cks             (string data_string, int data_read, int data_expect);
    void            tok             (string msg_string, double fdata_read, double fdata_expect, double tolerance, int status);
    void            inquire         (string prompt, const int &minv, const int &maxv, const int &radix, int &now);
    void            inquir2         (string prompt, const int &minv, const int &maxv, const int &radix, int &num, int &now);
    void            inquirl         (string prompt, const int &minv, const int &maxv, const int &radix, long int &now);
    void            inquirb         (string prompt, bool &now);

    long int        vme_open        ();
    long int        vme_read        (unsigned long &adr, unsigned short &rd_data);
    long int        vme_write       (unsigned long &adr, unsigned short &wr_data);
    long int        vme_sysreset    ();
    long int        vme_close       ();
    long int        vme_errs        (const int &print_mode);

    void            i4_to_tdi       (long int &i4, char  tdi[], const int &nbits, const int &spi);
    void            tdi_to_i4       (char  tdi[], long int &i4, const int &nbits, const int &spi);
    void            bit_to_array    (const int &idata, int iarray[], const int &n);

    void            vme_jtag_anystate_to_rti(unsigned long &adr, int &ichain);
    void            vme_jtag_write_ir       (unsigned long &adr, int &ichain, int &chip_id, int &opcode);
    void            vme_jtag_write_dr       (unsigned long &adr, int &ichain, int &chip_id, char wr_data[], char rd_data[], int &nbits);
    bool            vme_jtag_cable_detect   (unsigned long &base_adr);
    void            vme_jtag_io_byte        (unsigned long &adr, int &ichain, int &nframes, char tms[], char tdi[], char tdo[], const int &step_mode);

//------------------------------------------------------------------------------
// File scope declarations
//------------------------------------------------------------------------------
// VME calls
    long            status;
    unsigned long   boot_adr;
    unsigned long   adr;
    unsigned short  rd_data;
    unsigned short  wr_data;

//------------------------------------------------------------------------------
// Local
//------------------------------------------------------------------------------
    int             islot;
    int             geo_adr;

// ID reg
    int             id_rev;
    int             id_rev_day;
    int             id_rev_month;
    int             id_rev_year;
    int             id_rev_fpga;

    int             id_slot;
    int             id_ver;
    int             id_type;
    int             id_month;
    int             id_day;
    int             id_year;

// Firmware
    unsigned short  firmware_type;
    unsigned short  firmware_series;
    unsigned short  firmware_normal         =0xC;
    unsigned short  firmware_debug          =0xD;
    unsigned short  firmware_series_etype   =0xE;

    unsigned short  ise_version;
    unsigned short  ise_sub;
    unsigned short  ise_sp;

    char            timestr[9];
    char            datestr[9];

    string          tmb_type;
    string          fpga_series;
    string          firmware_name;
    string          scomputer_name;

// CSC type
    int             csc_me1ab;
    int             stagger_hs_csc;
    int             reverse_hs_csc;
    int             reverse_hs_me1a;
    int             reverse_hs_me1b;
    int             csc_type;
    string          flags_ok;

// Menu
    char            line[80];
    int             ifunc;
    int             i,j,k,n;

// Peek/poke
    char            rwe;
    unsigned long   newadr;
    unsigned long   newdata;

    int             nbang;
    int             ibang;
    bool            bang_read  = false;
    bool            bang_write = false;

// Misc
    int             boot_decode[16];
    unsigned short  boot_data_initial;

//------------------------------------------------------------------------------
    int main()
{
//------------------------------------------------------------------------------
//  Open Files
//------------------------------------------------------------------------------
// Log file
    log_file_name = "vmetst_log.txt";
    log_file      = fopen(log_file_name.c_str(),"w");
//  setbuf(log_file, NULL); // stops buffering, but is 3x slower

    if (log_file  != NULL)  fprintf(stdout,"\tOpened      %s\n",log_file_name.c_str());
    if (log_file  == NULL) {fprintf(stdout,"\tFailed to open %s\n",log_file_name.c_str()); pause("WTF?");}
    jtag_file=log_file;

// Put date into log files
    _strtime(timestr);
    _strdate(datestr);

    fprintf(stdout,"\tStarted:    %s %s\n",datestr,timestr);
    fprintf(log_file,"Started:    %s %s\n",datestr,timestr);

//------------------------------------------------------------------------------
//  Open VME Interface
//------------------------------------------------------------------------------
    bool vme_opened=false;

    status    = vme_open();                 // Connect to Bit3 VME controller
    islot     = tmb_global_slot;            // TMB VME slot address
    base_adr  = islot << 19;                // VME base address  for this slot
    boot_adr  = base_adr | tmb_boot_adr;    // VME boot register for this slot
    vme_opened= (status==0);

//------------------------------------------------------------------------------
// Get computer name
//------------------------------------------------------------------------------
    const int       infoBuflen=32;
    TCHAR           infoBuf[infoBuflen];
    char            cinfoBuf[infoBuflen];
    unsigned long   bufCharCount;

    bufCharCount = infoBuflen;                              // Initial length
    GetComputerName(infoBuf,&bufCharCount);                 // Returns actual length
    if (bufCharCount>=infoBuflen) infoBuf[infoBuflen-1]=0;  // Manually null terminate

    for (i=0; i<infoBuflen; ++i) (i<signed(bufCharCount)) ? cinfoBuf[i]=char(infoBuf[i]) : cinfoBuf[i]=0;
    scomputer_name = string(cinfoBuf);

//------------------------------------------------------------------------------
//  Get firmware type code
//------------------------------------------------------------------------------
begin:
    adr    = base_adr;
    status = vme_read(adr,rd_data);
    firmware_type   = (rd_data >> 0) & 0xF;
    firmware_series = (rd_data >> 4) & 0xF;
    geo_adr         = (rd_data >> 8) & 0x1F;

    tmb_type = "TMB2XXX";               //any non-tmb2005 board
    if (firmware_series == firmware_series_etype) tmb_type= "TMB2005";

// Decode id_rev
    adr    = base_adr+6;
    status = vme_read(adr,rd_data);
    id_rev          = rd_data;
    id_rev_day      = (id_rev >>  0) & 0x001F;
    id_rev_month    = (id_rev >>  5) & 0x000F;
    id_rev_year     = (id_rev >>  9) & 0x000F;
    id_rev_fpga     = (id_rev >> 13) & 0x0007;
    id_rev_fpga     =  id_rev_fpga+2;

// Get FPGA series
    if (id_rev_fpga==5) id_rev_fpga=6;

    fpga_series="Unknown";
    if (id_rev_fpga==4) fpga_series="XC2V4000";
    if (id_rev_fpga==6) fpga_series="XC6VLX195T";

// Get compiler version
    adr    = base_adr + ise_version_adr;    // Recent TMBs have ISE register
    status = vme_read(adr,rd_data);
    ise_version=(rd_data >> 8) & 0xFF;
    ise_sub    =(rd_data >> 4) & 0xF;
    ise_sp     =(rd_data >> 0) & 0xF;

    if (ise_version<8 && ise_version!=4){   // Old TMBs were all FND4.2sp3 and do not have an ISE register
    ise_version=4;
    ise_sub=2;
    ise_sp=3;
    }

    if (firmware_type == firmware_debug) {  // Debug TMBs are all ISE8.2sp3 and do not have an ISE register
    ise_version=8;
    ise_sub=2;
    ise_sp=3;
    }

// Determine firmware type
    firmware_name = "TMB200x type unknown...beware";                            // Unknown TMB type

    if      (id_rev_fpga==3||id_rev_fpga==4||id_rev_fpga==6) {
    if      (firmware_type==firmware_debug ) firmware_name = "Debug Loopback";  // Debug TMB type
    else if (firmware_type==firmware_normal) firmware_name = "Normal        ";  // Normal TMB type
    }

// Display tmb type
    fprintf(log_file,"Firmware    %s\n",firmware_name.c_str());
    fprintf(log_file,"RevCode     %4.4X=%2.2i/%2.2i/%2.2i series %1i\n",id_rev,id_rev_month,id_rev_day,id_rev_year,id_rev_fpga);
    fprintf(log_file,"ISE Version %2.2X.%1X SP%1.1X\n",ise_version,ise_sub,ise_sp);
    fprintf(log_file,"TMB Slot    [%2.2i] adr=%6.6X\n",islot,base_adr);
    fprintf(log_file,"C++ Compile %s\n\n",__TIMESTAMP__);

//------------------------------------------------------------------------------
//  Main Menu
//------------------------------------------------------------------------------
main_menu:
    printf("\tTMB Test Menu Host %s\n",scomputer_name.c_str());
    printf("\tTMB Type      %s\n",tmb_type.c_str());
    printf("\tFirmware      %s\n",firmware_name.c_str());
    printf("\tRevCode       %4.4X=%2.2i/%2.2i/%2.2i\n",id_rev,id_rev_month,id_rev_day,id_rev_year);
    printf("\tFPGA Series   %s\n",fpga_series.c_str());
    printf("\tISE  Version  %2.2X.%1X SP%1.1X\n",ise_version,ise_sub,ise_sp);
    printf("\tC++  Compiled %s\n\n",__TIMESTAMP__);

    printf("\t1:   Set ALCT ASIC delays\n");
    printf("\t3:   Read Boot Register\n");
    printf("\t4:   Peek/Poke Address\n");
    printf("\t7:   Hard Reset TMB\n");
    printf("\t8:   Hard Reset ALCT\n");
    printf("\t<cr> Exit\n");
    printf("       > ");

    gets(line);
    if (line[0]==NULL) goto exit;
    sscanf(line,"%i",&ifunc);

    i=abs(ifunc);
    if (i== 1) {void L100();    L100();     goto main_menu;    }
    if (i== 3) {void L300();    L300();     goto main_menu;}
    if (i== 4) {void L400();    L400();     goto main_menu;}
    if (i== 7) {void L700();    L700();     goto begin;    }
    if (i== 8) {void L800();    L800();     goto main_menu;}

    goto main_menu;

//------------------------------------------------------------------------------
//  Exit main: Close VME Interface
//------------------------------------------------------------------------------
exit:
    if (vme_opened    ) status = vme_close();   // Close VME
    if (log_file!=NULL) fclose(log_file);       // Close log file

    printf("\tSic transit gloria mundi");       // Say goodbye
    gets(line);

    return 0;
}
//------------------------------------------------------------------------------
//  Set ALCT ASIC delays
//------------------------------------------------------------------------------
    void L100() {
//L100:
    inquire("\tDelay[0-15] <cr>=%2i ", 0,15, 10, idelay);

// Get initial boot register
    status = vme_read(boot_adr,rd_data);    // Get current boot reg
    boot_data_initial = rd_data;
    printf("\tBoot=%4.4X Adr=%6.6X\n",rd_data,boot_adr);

    for (i=0; i<=15; ++i) {boot_decode[i]=(rd_data >> i) & 0x1;}

// Set jtag chain to ALCT user, chain source is boot register
    ichain    = 0x2;    // ALCT user jtag
    step_mode = 0;

// Set ASIC bank 0-3: 24 ASICs are grouped into 4 banks of 6 chips
    for (ibank=0;ibank<4;++ibank)
    {
    ibank_bit[0] = (ibank >> 0) & 0x1;
    ibank_bit[1] = (ibank >> 1) & 0x1;

    nrs_dly    = 1; //~Reset
    seltst_dly = 0; // Self test mode

    ccb_brcst_bit[0] = ibank_bit[0];    ccb_brcst_bit[4] = ibank_bit[0];    // 1st in time=2nd in time, alct not running 80MHz
    ccb_brcst_bit[1] = ibank_bit[1];    ccb_brcst_bit[5] = ibank_bit[1];
    ccb_brcst_bit[2] = nrs_dly;         ccb_brcst_bit[6] = nrs_dly;
    ccb_brcst_bit[3] = seltst_dly;      ccb_brcst_bit[7] = seltst_dly;

    ccb_brcst=0;
    for (i=0; i<8; ++i) ccb_brcst=ccb_brcst | (ccb_brcst_bit[i]<<i);

    ccb_cmd_wr = 0;
    ccb_cmd_wr = ccb_cmd_wr |  (0x1 << 0);          // Set ccb_cfg_wr[0]=1 to enable VME control of ccb_brcst
    ccb_cmd_wr = ccb_cmd_wr & ~(0x1 << 1);          // 1=chip deselected during shift in
    ccb_cmd_wr = ccb_cmd_wr & ~(0x1 << 2);
    ccb_cmd_wr = ccb_cmd_wr & ~(0x1 << 3);
    ccb_cmd_wr = ccb_cmd_wr |  (ccb_brcst << 8);    // Set new ccb_brcst[7:0]

    adr     = ccb_cmd_adr+base_adr;
    wr_data = ccb_cmd_wr;
    status  = vme_write(adr,wr_data);

// Set individual chip delays
    idelay_bit[0]=(idelay>>0) & 0x1;
    idelay_bit[1]=(idelay>>1) & 0x1;
    idelay_bit[2]=(idelay>>2) & 0x1;
    idelay_bit[3]=(idelay>>3) & 0x1;

    for (ichip=0;ichip<6;++ichip) {
    for (ibit =0;ibit <4;++ibit ) {
    T[ichip][ibit]=idelay_bit[ibit];
    }}

    for (ichip=0;ichip<6;++ichip)  {
    for (ichan=0;ichan<16;++ichan) {
    C[ichip][ichan]=0x0;
    }}

// Shift in 20 bits per chip x 6 chips: 1st T4,T3,T2,T1, C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16 last_in
    nframes = 120;
    for (i=0; i<nframes; ++i) tms[i]=0; // TMS= not used
    for (i=0; i<nframes; ++i) tdi[i]=0; // TDI= data bits T[4:1] and C[16:1]

    for (ichip=0;ichip<6;++ichip)
    {
    tdi[ichip*20+ 0]=T[ichip][3];
    tdi[ichip*20+ 1]=T[ichip][2];
    tdi[ichip*20+ 2]=T[ichip][1];
    tdi[ichip*20+ 3]=T[ichip][0];

    tdi[ichip*20+ 4]=C[ichip][0];
    tdi[ichip*20+ 5]=C[ichip][1];
    tdi[ichip*20+ 6]=C[ichip][2];
    tdi[ichip*20+ 7]=C[ichip][3];

    tdi[ichip*20+ 8]=C[ichip][4];
    tdi[ichip*20+ 9]=C[ichip][5];
    tdi[ichip*20+10]=C[ichip][6];
    tdi[ichip*20+11]=C[ichip][7];

    tdi[ichip*20+12]=C[ichip][8];
    tdi[ichip*20+13]=C[ichip][9];
    tdi[ichip*20+14]=C[ichip][10];
    tdi[ichip*20+15]=C[ichip][11];

    tdi[ichip*20+16]=C[ichip][12];
    tdi[ichip*20+17]=C[ichip][13];
    tdi[ichip*20+18]=C[ichip][14];
    tdi[ichip*20+19]=C[ichip][15];
    };

    printf("\ttdi=");for(i=0;i<nframes;++i)printf("%1i",tdi[i]);printf("\n");

// Send shift data to JTAG
    vme_jtag_io_byte(boot_adr,ichain,nframes,tms,tdi,tdo,step_mode);    // Shift in data
    status = vme_read(boot_adr,rd_data); tdo_bit0=(rd_data>>15)&0x1;    // Get TDO before 1st TCK
    vme_jtag_io_byte(boot_adr,ichain,nframes,tms,tdi,tdo,step_mode);    // Shift it in again to push out previous data

    printf("\ttdo=%1i",tdo_bit0);for(i=0;i<nframes-1;++i)printf("%1i",tdo[i]);printf("\n");

// Deassert chip select: shift register data transfers to FFs on rising edge of cs
    ccb_cmd_wr = ccb_cmd_wr | (0x1 << 1);   // 0=chip loads after shift in
    ccb_cmd_wr = ccb_cmd_wr | (0x1 << 2);
    ccb_cmd_wr = ccb_cmd_wr | (0x1 << 3);

    adr     = ccb_cmd_adr+base_adr;
    wr_data = ccb_cmd_wr;
    status  = vme_write(adr,wr_data);

    }   // close for ibank
// Done
    return;
}
//------------------------------------------------------------------------------
//  Read Boot Register
//------------------------------------------------------------------------------
    void L300() {
L300:

    status = vme_read(boot_adr,rd_data);    // Get current boot reg
    if (ifunc < 0) goto L300;               // Bang mode

    printf("\tBoot=%4.4X Adr=%6.6X\n",rd_data,boot_adr);

    for (i=0; i<=15; ++i) {
    boot_decode[i]=(rd_data >> i) & 0x1;
    }

    i=0;
    printf("\n");
    printf("\t[%2.2i]%2i  R/W jtag_vme1  (tdi) vme tdi\n",              i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W jtag_vme2  (tms) vme tms\n",              i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W jtag_vme3  (tck) vme tck\n",              i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W sel_vme0    00XX ALCT JTAG Chain\n",      i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W sel_vme1    01XX TMB Mezzanine FPGA\n",   i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W sel_vme2    10XX TMB User PROMs JTAG\n",  i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W sel_vme3    11XX TMB FPGA User JTAG\n",   i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W vme/usr_en  1=JTAG sourced by Boot\n",    i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W HardRstAlct 1=Hard reset to ALCT FPGA\n", i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W HardRstTmb  1=Hard reset to TMB FPGA\n",  i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W /EnResetAlct0=Allow TMB reset ALCT\n",    i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W /FpgaVmeEn  1=Allow TMB to issue VME\n",  i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W /MezClockEn 0=Enable TMB mez clock\n",    i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R/W HardResetRpc1=Hard reset to RAT\n",       i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R   vme_ready   1=FPGA vme logic ready\n",    i,boot_decode[i]); i++;
    printf("\t[%2.2i]%2i  R   jtag_vme0   (tdo) vme tdo\n",             i,boot_decode[i]); i++;

    printf("\n\t<cr> to continue:");
    gets(line);

    return;
}
//------------------------------------------------------------------------------
//  Peek/Poke an arbitrary address
//------------------------------------------------------------------------------
    void L400() {
L400:

    bang_read =false;
    bang_write=false;

    printf("\tPeek/Poke(hex): read=r,adr | write=w,adr,wrdata | inc adr=<cr> | e=exit\n");
    adr=base_adr;

L420:
    rd_data=0xFFFF;     // clear out previous read, bit 3 doesn't update rd_data if no dtack
    status = vme_read(adr,rd_data);
    if (bang_read) goto L420;

    printf("\tadr=%6.6X data=%4.4X r/w/e,adr,wrdata=",adr,rd_data);
    gets(line);
    n=strlen(line);

// <cr> = increment address
    if (line[0]==NULL) {
    adr=adr+2;
    goto L420;
    }

// Parse input string into adr and wr_data
    if (n < 1) goto L400;
    sscanf(line,"%c,%X,%X",&rwe,&newadr,&newdata);

// e=exit
    if ((rwe=='e') || (rwe=='E')) return;
    if ((rwe=='R') || (rwe=='W')) printf("\tBang mode....\n");

//  Check adr is even and 24 bits or less and data is 16 bits or less
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
//  Hard Reset TMB
//------------------------------------------------------------------------------
    void L700() {
L700:

    status  = vme_read(boot_adr,rd_data);           // Get current boot reg
    wr_data = rd_data | 0x0200;                     // Turn on  TMB hard reset
    status  = vme_write(boot_adr,wr_data);          // Assert   TMB hard reset

    wr_data = rd_data & ~0x0200;                    // Turn off TMB hard reset
    status  = vme_write(boot_adr,wr_data);          // Restore boot reg
//  sleep(150);                                     // Wait for TMB to reload, Virtex2 takes 100ms
    sleep(450);                                     // Wait for TMB to reload, Virtex6 takes 400ms
    if (ifunc <0 ) goto L700;                       // Bang mode
    return;                                         // Get latest firmware type
}
//------------------------------------------------------------------------------
//  Hard Reset ALCT
//------------------------------------------------------------------------------
    void L800() {
L800:

    status  = vme_read(boot_adr,rd_data);           // Get current boot reg
    wr_data = rd_data | 0x0100;                     // Turn on  ALCT hard reset
    status  = vme_write(boot_adr,wr_data);          // Assert   ALCT hard reset

    wr_data = rd_data & ~0x0100;                    // Turn off ACLT hard reset
    status  = vme_write(boot_adr,wr_data);          // Restore boot reg
    sleep(150);                                     // Wait for ALCT to reload
    if (ifunc < 0) goto L800;                       // Bang mode
    return;
}
//------------------------------------------------------------------------------
//
// Service routines for main
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  Check data read vs data expected
//------------------------------------------------------------------------------
    void ck(string msg_string, int data_read, int data_expect)
{
    if (data_read != data_expect) {
    fprintf(stdout,  "ERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
    fprintf(log_file,"ERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
//  pause(" ");
    }
    return;
}
//------------------------------------------------------------------------------
//  Check data read vs data expected, with status return
//------------------------------------------------------------------------------
    int cks(string msg_string, int data_read, int data_expect)
{
    int status;

    status = 0; // good return
    if (data_read != data_expect) {
    status = 1; // bad return
    fprintf(stdout,  "\tERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
    fprintf(log_file,"\tERRm: in %s: read=%8.8X expect=%8.8X\n",msg_string.c_str(),data_read,data_expect);
//  pause(" ");
    }

    return status;
}
//------------------------------------------------------------------------------
//  Check data read vs data expected, floating point version  with tolerance
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
//  Inquire prompt for integer
//------------------------------------------------------------------------------
    void inquire(string prompt, const int &minv, const int &maxv, const int &radix, int &now)
{
    char    line[80];
    int     i;
    int     n;

ask:
    printf(prompt.c_str(),now);
    gets(line);
    n=strlen(line);
    if (radix==16)  sscanf(line,"%x",&i);
    else            sscanf(line,"%i",&i);

    if ((n!=0) && ((i<minv) || (i>maxv))) {
    if (radix==16) printf("Out of range. Expect %X to %X\n",minv,maxv);
    else           printf("Out of range. Expect %i to %i\n",minv,maxv);
    goto ask;
    }
    if (n!=0) now = i;
    return;
}
//------------------------------------------------------------------------------
//  Inquire prompt for two integers
//------------------------------------------------------------------------------
    void inquir2(string prompt, const int &min, const int &max, const int &radix, int &num, int &now)
{
    char    line[80];
    int     i;
    int     n;

ask:
    printf(prompt.c_str(),num,now);
    gets(line);
    n=strlen(line);
    if (radix==16)  sscanf(line,"%x",&i);
    else            sscanf(line,"%i",&i);

    if ((n!=0) && ((i<min) || (i>max))) {
    if (radix==16) printf("Out of range. Expect %X to %X\n",min,max);
    else           printf("Out of range. Expect %i to %i\n",min,max);
    goto ask;
    }
    if (n!=0) now = i;
    return;
}
//------------------------------------------------------------------------------
//  Inquire prompt for long integer
//------------------------------------------------------------------------------
    void inquirl(string prompt, const int &min, const int &max, const int &radix, long int &now)
{
    char        line[80];
    long int    i;
    int         n;

ask:
    printf(prompt.c_str(),now);
    gets(line);
    n=strlen(line);
    if (radix==16)  sscanf(line,"%x",&i);
    else            sscanf(line,"%i",&i);

    if ((n!=0) && ((i<min) || (i>max))) {
    if (radix==16) printf("Out of range. Expect %X to %X\n",min,max);
    else           printf("Out of range. Expect %i to %i\n",min,max);
    goto ask;
    }
    if (n!=0) now = i;
    return;
}
//------------------------------------------------------------------------------
//  Inquire prompt for bool
//------------------------------------------------------------------------------
    void inquirb(string prompt, bool &now)
{
    char    line[80];
    char    i;
    int     n;

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
//  Pass Fail prompt
//------------------------------------------------------------------------------
    bool pass_fail(string prompt)
{
    char    line[80];
    int     i;
    int     n;
    bool    ans;

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
//  Display Test OK
//------------------------------------------------------------------------------
    void aok(string msg_string)
{
    int     tab           = 45;
    int     msg_spaces    = msg_string.length();
    int     insert_spaces = tab-msg_spaces;
    string  spaces        = " ";

    for (int i=0; i<=insert_spaces; ++i) spaces.append(string(" "));

    fprintf(log_file,"\t%s %s OK\n",msg_string.c_str(),spaces.c_str()); // log file
    fprintf(stdout,  "\t%s %s OK\n",msg_string.c_str(),spaces.c_str()); // screen
    return;
}
//------------------------------------------------------------------------------
//  Display Test OK or FAIL
//------------------------------------------------------------------------------
    void aokf(string msg_string, const int itest, const int status)
{
    string  sstat[2]={"FAIL","PASS"};

    int     tab           = 45;
    int     msg_spaces    = msg_string.length();
    int     insert_spaces = tab-msg_spaces;
    string  spaces        = " ";

    for (int i=0; i<=insert_spaces; ++i) spaces.append(string(" "));

    fprintf(log_file,"\t%3i %s %s %s\n",itest,msg_string.c_str(),spaces.c_str(),sstat[status].c_str()); // log file
    fprintf(stdout,  "\t%3i %s %s %s\n",itest,msg_string.c_str(),spaces.c_str(),sstat[status].c_str()); // screen

    return;
}
//------------------------------------------------------------------------------
//  Convert integer bit string to an array
//------------------------------------------------------------------------------
    void bit_to_array(const int &idata, int iarray[], const int &n) {
    int i;

    for (i=0; i<n; ++i) {
    iarray[i]=(idata >> i) & 0x00000001;
    }
    return;
}
//--------------------------------------------------------------------------------
// Wait for specified number of milliseconds, probably MS Visual Studio specific
//--------------------------------------------------------------------------------
    void sleep(clock_t msec)
{
   clock_t goal;
   goal = msec + clock();
   while (goal > clock());
}

//------------------------------------------------------------------------------
// The bitter end
//------------------------------------------------------------------------------
