//------------------------------------------------------------------------------------------
//	Decodes a Xilinx FPGA or PROM IDCODE
//
//	04/13/12 Initial
//
//------------------------------------------------------------------------------------------
// IDCODEs:
//
//	Device			IR Length	IDCODEinstruction	DR Length	IDCODE			USERCODEinstruction	DR Length
//	XC2V4000		6 bits		0x09				32 bits		0xv1050093		0x08				32 bits
//	XC2V4000.AGT	6 bits		0x09				32 bits		0x11050093		0x08				32 bits
//	XCV600E			5 bits		0x09				32 bits		0xv0A30093		0x08				32 bits
//	XC6SLX150.BIV	6 bits		0x09				32 bits		0xv401D093		0x08				32 bits
//	XC6SLX150.CIV	6 bits		0x09				32 bits		0x4401D093		0x08				32 bits
//	XCS40XL			3 bits		0x6					32 bits		0x0041C093		None				None
//	XC2S50E			5 bits		0x09				32 bits		0x00610093		0x08				32 bits
//	XC2S50E.AGT		5 bits		0x09				32 bits		0x20A10093		0x08				32 bits
//	XC18V01			8 bits		0xFE				32 bits		0xv50X4093		0xFD				32 bits
//	XC18V01.AEN		8 bits		0xFE				32 bits		0x05004093		0xFD				32 bits
//	XC18V01.BRT		8 bits		0xFE				32 bits		0xF5034093		0xFD				32 bits
//	XC18V04			8 bits		0xFE				32 bits		0xv50X6093		0xFD				32 bits
//	XC18V04.AEN		8 bits		0xFE				32 bits		0x05026093		0xFD				32 bits
//	XC18V04.ART		8 bits		0xFE				32 bits		0x05036093		0xFD				32 bits
//	XC18V04.BRT		8 bits		0xFE				32 bits		0xF5036093		0xFD				32 bits
//	XC18V256		8 bits		0xFE				32 bits		0x05022093		0xFD				32 bits
//	XC18V512.AEN	8 bits		0xFE				32 bits		0x05023093		0xFD				32 bits
//	XC18V512.ART	8 bits		0xFE				32 bits		0x05033093		0xFD				32 bits
//	XCF08P			16 bits		0x00FE				32 bits		0xv5057093		0x00FD				32 bits
//	XCF08P.BRT		16 bits		0x00FE				32 bits		0xE5057093		0x00FD				32 bits
//	XCF08P.BTT		16 bits		0x00FE				32 bits		0xD5057093		0x00FD				32 bits
//	XCF32P			16 bits		0x00FE				32 bits		0xv5059093		0x00FD				32 bits
//	XCF32P.ATT		16 bits		0x00FE				32 bits		0xD5059093		0x00FD				32 bits
//
//------------------------------------------------------------------------------------------
// Headers
//------------------------------------------------------------------------------------------
	#include <stdio.h>
	#include <iostream>
	using namespace std;

//------------------------------------------------------------------------------------------
	void idcode_decode
	(
	long	&idcode,
	string	&sdevice_type,
	string	&sdevice_name,
	string	&sdevice_version,
	string	&sdevice_size
	)
//------------------------------------------------------------------------------------------
{
// Initialize return fields
	sdevice_type	= "TYPE";
	sdevice_name	= "DEVNAME";
	sdevice_version = "VER";
	sdevice_size	= "SIZE";

// Interpret ID code: 0x0FF0FFF finds the device for any rev, 0xFFFFFFFF locates the specific rev	WTFs are devices I dont know the version for yet
	if ((idcode & 0x0FFFFFFF) == 0x01050093) {sdevice_name = "XC2V4000";	sdevice_version = "UNK"; sdevice_type="FPGA"; sdevice_size="46K ";}
	if ((idcode & 0xFFFFFFFF) == 0x11050093) {sdevice_name = "XC2V4000";	sdevice_version = "AGT"; sdevice_type="FPGA"; sdevice_size="46K ";}

	if ((idcode & 0x0FFFFFFF) == 0x00A30093) {sdevice_name = "XCV600E";		sdevice_version = "UNK"; sdevice_type="FPGA"; sdevice_size="9K  ";}
	if ((idcode & 0xFFFFFFFF) == 0x90A30093) {sdevice_name = "XCV600E";		sdevice_version = "AFS"; sdevice_type="FPGA"; sdevice_size="9K  ";}

	if ((idcode & 0x0FFFFFFF) == 0x0401D093) {sdevice_name = "XC6SLX150";	sdevice_version = "UNK"; sdevice_type="FPGA"; sdevice_size="150K";}
	if ((idcode & 0xFFFFFFFF) == 0x3401D093) {sdevice_name = "XC6SLX150";	sdevice_version = "BIV"; sdevice_type="FPGA"; sdevice_size="150K";}
	if ((idcode & 0xFFFFFFFF) == 0x4401D093) {sdevice_name = "XC6SLX150";	sdevice_version = "CIV"; sdevice_type="FPGA"; sdevice_size="150K";}

	if ((idcode & 0x0FFFFFFF) == 0x0041C093) {sdevice_name = "XCS40XL";		sdevice_version = "UNK"; sdevice_type="FPGA"; sdevice_size="2K  ";}
	if ((idcode & 0xFFFFFFFF) == 0x0041C093) {sdevice_name = "XCS40XL";		sdevice_version = "AKP"; sdevice_type="FPGA"; sdevice_size="2K  ";}
	if ((idcode & 0xFFFFFFFF) == 0x0041C093) {sdevice_name = "XCS40XL";		sdevice_version = "AKP"; sdevice_type="FPGA"; sdevice_size="2K  ";}

	if ((idcode & 0x0F6FFFFF) == 0x00610093) {sdevice_name = "XC2S50E";		sdevice_version = "UNK"; sdevice_type="FPGA"; sdevice_size="2K  ";}
	if ((idcode & 0xFFFFFFFF) == 0x20A10093) {sdevice_name = "XC2S50E";		sdevice_version = "AGT"; sdevice_type="FPGA"; sdevice_size="2K  ";}

	if ((idcode & 0x0FF0FFFF) == 0x05004093) {sdevice_name = "XC18V01";		sdevice_version = "UNK"; sdevice_type="PROM"; sdevice_size="1M  ";}
	if ((idcode & 0xFFFFFFFF) == 0x05034093) {sdevice_name = "XC18V01";		sdevice_version = "AEN"; sdevice_type="PROM"; sdevice_size="1M  ";}
	if ((idcode & 0xFFFFFFFF) == 0x05044093) {sdevice_name = "XC18V01";		sdevice_version = "BRT"; sdevice_type="PROM"; sdevice_size="1M  ";}
	if ((idcode & 0xFFFFFFFF) == 0xF5034093) {sdevice_name = "XC18V01";		sdevice_version = "BRT"; sdevice_type="PROM"; sdevice_size="1M  ";}

	if ((idcode & 0x0FF0FFFF) == 0x05006093) {sdevice_name = "XC18V04";		sdevice_version = "UNK"; sdevice_type="PROM"; sdevice_size="4M  ";}
	if ((idcode & 0xFFFFFFFF) == 0x05026093) {sdevice_name = "XC18V04";		sdevice_version = "AEN"; sdevice_type="PROM"; sdevice_size="4M  ";}
	if ((idcode & 0xFFFFFFFF) == 0x05036093) {sdevice_name = "XC18V04";		sdevice_version = "ART"; sdevice_type="PROM"; sdevice_size="4M  ";}
	if ((idcode & 0xFFFFFFFF) == 0xF5036093) {sdevice_name = "XC18V04";		sdevice_version = "BRT"; sdevice_type="PROM"; sdevice_size="4M  ";}

	if ((idcode & 0x0FF0FFFF) == 0x05022093) {sdevice_name = "XC18V256";	sdevice_version = "UNK"; sdevice_type="PROM"; sdevice_size="256K";}	
	if ((idcode & 0xFFFFFFFF) == 0x05022093) {sdevice_name = "XC18V256";	sdevice_version = "STD"; sdevice_type="PROM"; sdevice_size="256K";}		

	if ((idcode & 0x0FF0FFFF) == 0x05023093) {sdevice_name = "XC18V512";	sdevice_version = "UNK"; sdevice_type="PROM"; sdevice_size="512K";}	
	if ((idcode & 0xFFFFFFFF) == 0x05023093) {sdevice_name = "XC18V512";	sdevice_version = "AEN"; sdevice_type="PROM"; sdevice_size="512K";}	
	if ((idcode & 0xFFFFFFFF) == 0x05033093) {sdevice_name = "XC18V512";	sdevice_version = "ART"; sdevice_type="PROM"; sdevice_size="512K";}	

	if ((idcode & 0x0FFFFFFF) == 0x05057093) {sdevice_name = "XCF08P";		sdevice_version = "UNK"; sdevice_type="PROM"; sdevice_size="8M  ";}
	if ((idcode & 0xFFFFFFFF) == 0xE5057093) {sdevice_name = "XCF08P";		sdevice_version = "BRT"; sdevice_type="PROM"; sdevice_size="8M  ";}
	if ((idcode & 0xFFFFFFFF) == 0xD5057093) {sdevice_name = "XCF08P";		sdevice_version = "BTT"; sdevice_type="PROM"; sdevice_size="8M  ";}

	if ((idcode & 0x0FFFFFFF) == 0x05059093) {sdevice_name = "XCF32P";		sdevice_version = "UNK"; sdevice_type="PROM"; sdevice_size="32M ";}
	if ((idcode & 0xFFFFFFFF) == 0xD5059093) {sdevice_name = "XCF32P";		sdevice_version = "ATT"; sdevice_type="PROM"; sdevice_size="32M ";}

	return;
}
//------------------------------------------------------------------------------------------
// End idcode_decode.v
//------------------------------------------------------------------------------------------
