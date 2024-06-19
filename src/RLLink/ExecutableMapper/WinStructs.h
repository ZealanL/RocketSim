#pragma once

#include <stdint.h>

// Stuff in here is a modified copy of structs from https://github.com/jonasstrandstedt/MinGW/blob/master/MinGW/include/winnt.h

struct IMAGE_DOS_HEADER {
	uint16_t e_magic;
	uint16_t e_cblp;
	uint16_t e_cp;
	uint16_t e_crlc;
	uint16_t e_cparhdr;
	uint16_t e_minalloc;
	uint16_t e_maxalloc;
	uint16_t e_ss;
	uint16_t e_sp;
	uint16_t e_csum;
	uint16_t e_ip;
	uint16_t e_cs;
	uint16_t e_lfarlc;
	uint16_t e_ovno;
	uint16_t e_res[4];
	uint16_t e_oemid;
	uint16_t e_oeminfo;
	uint16_t e_res2[10];
	int32_t e_lfanew;
};

struct IMAGE_FILE_HEADER {
	uint16_t machine;
	uint16_t numberOfSections;
	uint32_t timeDateStamp;
	uint32_t pointerToSymbolTable;
	uint32_t numberOfSymbols;
	uint16_t sizeOfOptionalHeader;
	uint16_t characteristics;
};

#define IMAGE_FILE_RELOCS_STRIPPED 0x0001
#define IMAGE_FILE_EXECUTABLE_IMAGE 0x0002
#define IMAGE_FILE_LINE_NUMS_STRIPPED 0x0004
#define IMAGE_FILE_LOCAL_SYMS_STRIPPED 0x0008
#define IMAGE_FILE_AGGRESIVE_WS_TRIM 0x0010
#define IMAGE_FILE_LARGE_ADDRESS_AWARE 0x0020
#define IMAGE_FILE_BYTES_REVERSED_LO 0x0080
#define IMAGE_FILE_32BIT_MACHINE 0x0100
#define IMAGE_FILE_DEBUG_STRIPPED 0x0200
#define IMAGE_FILE_REMOVABLE_RUN_FROM_SWAP 0x0400
#define IMAGE_FILE_NET_RUN_FROM_SWAP 0x0800
#define IMAGE_FILE_SYSTEM 0x1000
#define IMAGE_FILE_DLL 0x2000
#define IMAGE_FILE_UP_SYSTEM_ONLY 0x4000
#define IMAGE_FILE_BYTES_REVERSED_HI 0x8000

#define IMAGE_FILE_MACHINE_AMD64 0x8664

struct IMAGE_DATA_DIRECTORY {
	uint32_t virtualAddress;
	uint32_t size;
};

#define IMAGE_NUMBEROF_DIRECTORY_ENTRIES 16

struct IMAGE_OPTIONAL_HEADER64 {
	uint16_t magic;
	uint8_t  majorLinkerVersion;
	uint8_t  minorLinkerVersion;
	uint32_t sizeOfCode;
	uint32_t sizeOfInitializedData;
	uint32_t sizeOfUninitializedData;
	uint32_t addressOfEntryPoint;
	uint32_t baseOfCode;
	uint64_t imageBase;
	uint32_t sectionAlignment;
	uint32_t fileAlignment;
	uint16_t majorOperatingSystemVersion;
	uint16_t minorOperatingSystemVersion;
	uint16_t majorImageVersion;
	uint16_t minorImageVersion;
	uint16_t majorSubsystemVersion;
	uint16_t minorSubsystemVersion;
	uint32_t win32VersionValue;
	uint32_t sizeOfImage;
	uint32_t sizeOfHeaders;
	uint32_t checkSum;
	uint16_t subsystem;
	uint16_t dllCharacteristics;
	uint64_t sizeOfStackReserve;
	uint64_t sizeOfStackCommit;
	uint64_t sizeOfHeapReserve;
	uint64_t sizeOfHeapCommit;
	uint32_t loaderFlags;
	uint32_t numberOfRvaAndSizes;
	IMAGE_DATA_DIRECTORY dataDirectory[IMAGE_NUMBEROF_DIRECTORY_ENTRIES];
};
typedef IMAGE_OPTIONAL_HEADER64 IMAGE_OPTIONAL_HEADER;

#define IMAGE_NT_OPTIONAL_HDR32_MAGIC 0x10b
#define IMAGE_NT_OPTIONAL_HDR64_MAGIC 0x20b
#define IMAGE_ROM_OPTIONAL_HDR_MAGIC 0x107


#define IMAGE_NT_OPTIONAL_HDR_MAGIC IMAGE_NT_OPTIONAL_HDR64_MAGIC

struct IMAGE_NT_HEADERS64 {
	uint32_t signature;
	IMAGE_FILE_HEADER fileHeader;
	IMAGE_OPTIONAL_HEADER64 optionalHeader;
};
typedef IMAGE_NT_HEADERS64 IMAGE_NT_HEADERS;

struct IMAGE_BASE_RELOCATION {
	uint32_t virtualAddress;
	uint32_t sizeOfBlock;
};

#define IMAGE_SUBSYSTEM_UNKNOWN 0
#define IMAGE_SUBSYSTEM_NATIVE 1
#define IMAGE_SUBSYSTEM_WINDOWS_GUI 2
#define IMAGE_SUBSYSTEM_WINDOWS_CUI 3
#define IMAGE_SUBSYSTEM_NATIVE_WINDOWS 8
#define IMAGE_SUBSYSTEM_WINDOWS_CE_GUI 9

#define IMAGE_DLLCHARACTERISTICS_HIGH_ENTROPY_VA 0x0020 
#define IMAGE_DLLCHARACTERISTICS_DYNAMIC_BASE 0x0040 
#define IMAGE_DLLCHARACTERISTICS_FORCE_INTEGRITY 0x0080 
#define IMAGE_DLLCHARACTERISTICS_NX_COMPAT 0x0100 
#define IMAGE_DLLCHARACTERISTICS_NO_ISOLATION 0x0200 
#define IMAGE_DLLCHARACTERISTICS_NO_SEH 0x0400 
#define IMAGE_DLLCHARACTERISTICS_NO_BIND 0x0800 
#define IMAGE_DLLCHARACTERISTICS_APPCONTAINER 0x1000 
#define IMAGE_DLLCHARACTERISTICS_WDM_DRIVER 0x2000 
#define IMAGE_DLLCHARACTERISTICS_GUARD_CF 0x4000
#define IMAGE_DLLCHARACTERISTICS_TERMINAL_SERVER_AWARE 0x8000

#define IMAGE_DIRECTORY_ENTRY_EXPORT 0
#define IMAGE_DIRECTORY_ENTRY_IMPORT 1
#define IMAGE_DIRECTORY_ENTRY_RESOURCE 2
#define IMAGE_DIRECTORY_ENTRY_EXCEPTION 3
#define IMAGE_DIRECTORY_ENTRY_SECURITY 4
#define IMAGE_DIRECTORY_ENTRY_BASERELOC 5
#define IMAGE_DIRECTORY_ENTRY_DEBUG 6

#define IMAGE_DIRECTORY_ENTRY_ARCHITECTURE 7
#define IMAGE_DIRECTORY_ENTRY_GLOBALPTR 8
#define IMAGE_DIRECTORY_ENTRY_TLS 9
#define IMAGE_DIRECTORY_ENTRY_LOAD_CONFIG 10
#define IMAGE_DIRECTORY_ENTRY_BOUND_IMPORT 11
#define IMAGE_DIRECTORY_ENTRY_IAT 12
#define IMAGE_DIRECTORY_ENTRY_DELAY_IMPORT 13
#define IMAGE_DIRECTORY_ENTRY_COM_DESCRIPTOR 14

struct IMAGE_SECTION_HEADER {
	uint8_t name[8];
	union {
		uint32_t physicalAddress;
		uint32_t virtualSize;
	} misc;
	uint32_t virtualAddress;
	uint32_t sizeOfRawData;
	uint32_t pointerToRawData;
	uint32_t pointerToRelocations;
	uint32_t pointerToLinenumbers;
	uint16_t numberOfRelocations;
	uint16_t numberOfLinenumbers;
	uint32_t characteristics;
};