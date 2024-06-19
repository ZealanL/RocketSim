#include "ExecutableMapper.h"

#include "WinStructs.h"
#include "../MemoryUtil/MemoryUtil.h"

RS_NS_START

template <typename T>
T* GetBinMem(byte* mem, size_t memSize, size_t offset) {
	if (offset + sizeof(T) > memSize) {
		RS_ERR_CLOSE(
			"ExecutableMapper: Failed to read " << typeid(T).name() << " from binary at offset 0x" << std::hex << offset << " with size 0x" << std::hex << sizeof(T) <<
			", exceeded binary size of 0x" << std::hex << memSize
		);
	}

	return (T*)(mem + offset);
}

void ExecutableMapper::Map(byte* exeFileMem, size_t exeFileSize, byte*& memoryOut, size_t& sizeOut, byte*& codeBaseOut, size_t& codeSizeOut) {
#define GETM(type, offset) GetBinMem<type>(exeFileMem, exeFileSize, offset)
#define READM(type, offset) (*GETM(type, offset))

	constexpr const char* ERR_PREFIX = "ExecutableMapper::Map(): ";

	RS_LOG(ERR_PREFIX << "Mapping...");

	auto dos = GETM(IMAGE_DOS_HEADER, 0);
	if (dos->e_magic != 'ZM')
		RS_ERR_CLOSE(ERR_PREFIX << "Invalid DOS header magic: 0x" << std::hex << dos->e_magic);

	auto ntHeader = GETM(IMAGE_NT_HEADERS, dos->e_lfanew);
	auto fileHeader = &ntHeader->fileHeader;

	if (fileHeader->machine != IMAGE_FILE_MACHINE_AMD64)
		RS_ERR_CLOSE(ERR_PREFIX << "Invalid file header machine id: 0x" << std::hex << fileHeader->machine);

	auto optionalHeader = &ntHeader->optionalHeader;
	
	{ // Update size and allocate memory
		sizeOut = optionalHeader->sizeOfImage;

		int alignment = 1024 * 8; // Most PCs use 4KB but we'll align to 8KB just to be safe
		int allocSize = sizeOut;
		if (allocSize % alignment != 0)
			allocSize += alignment - (sizeOut % alignment);

#ifdef _MSC_VER
		memoryOut = (byte*)_mm_malloc(allocSize, alignment);
#else
		memoryOut = (byte*)aligned_alloc(allocSize, alignment);
#endif

		memset(memoryOut, 0xCC, allocSize);
	}
	
	{ // Map headers (just copy)
		if (optionalHeader->sizeOfHeaders > exeFileSize || optionalHeader->sizeOfHeaders > sizeOut)
			RS_ERR_CLOSE(ERR_PREFIX << "Invalid headers size: 0x" << std::hex << sizeOut);

		memcpy(memoryOut, exeFileMem, optionalHeader->sizeOfHeaders);
	}
	
	{ // Map each section
		codeBaseOut = NULL;
		codeSizeOut = 0;

		constexpr int MAX_SECTIONS = 256;
		if (fileHeader->numberOfSections <= 0 || fileHeader->numberOfSections > MAX_SECTIONS)
			RS_ERR_CLOSE(ERR_PREFIX << "Invalid file header section count: " << fileHeader->numberOfSections);

		for (int i = 0; i < fileHeader->numberOfSections; i++) {
			auto sectionHeader = GETM(IMAGE_SECTION_HEADER, dos->e_lfanew + sizeof(IMAGE_NT_HEADERS) + i*sizeof(IMAGE_SECTION_HEADER));
			
			uint32_t rawEndAddr = sectionHeader->pointerToRawData + sectionHeader->sizeOfRawData;
			uint32_t mappedEndAddr = sectionHeader->virtualAddress + sectionHeader->sizeOfRawData;
			if (rawEndAddr > exeFileSize || mappedEndAddr > sizeOut)
				RS_ERR_CLOSE(
					ERR_PREFIX << "Invalid section header, size: 0x" << std::hex << sectionHeader->sizeOfRawData <<
					", raw end address: 0x" << std::hex << rawEndAddr <<
					", mapped end address: 0x" << std::hex << mappedEndAddr);

			memcpy(memoryOut + sectionHeader->virtualAddress, exeFileMem + sectionHeader->pointerToRawData, sectionHeader->sizeOfRawData);

			bool isCode = strcmp((char*)sectionHeader->name, ".text") == 0;
			MemoryUtil::SetMemoryProtection(memoryOut + sectionHeader->virtualAddress, sectionHeader->sizeOfRawData, isCode);

			if (isCode) {
				codeBaseOut = memoryOut + sectionHeader->virtualAddress;
				codeSizeOut = sectionHeader->sizeOfRawData;
			}
		}
	}

#undef GETM // Update GETM to use mapped memory instead of file data
#define GETM(type, offset) GetBinMem<type>(memoryOut, sizeOut, offset)

	int relocationsDone = 0;
	{ // Resolve relocations

		uint32_t curRelocOffset = optionalHeader->dataDirectory[IMAGE_DIRECTORY_ENTRY_BASERELOC].virtualAddress;
		for (auto curRelocBlock = GETM(IMAGE_BASE_RELOCATION, curRelocOffset); curRelocBlock->virtualAddress;) {

			if (curRelocBlock->sizeOfBlock < sizeof(IMAGE_BASE_RELOCATION))
				RS_ERR_CLOSE(ERR_PREFIX << "Invalid relocation block, offset: 0x" << std::hex << curRelocOffset);

			if (curRelocOffset + curRelocBlock->sizeOfBlock > sizeOut) {
				RS_ERR_CLOSE(
					ERR_PREFIX << "Invalid relocation block, exceeds maximum address " <<
					"(0x" << std::hex << (curRelocBlock->virtualAddress + curRelocBlock->sizeOfBlock) << " > 0x" << std::hex << sizeOut << ")"
				);
			}

			uintptr_t newBase = (uintptr_t)memoryOut;

			uint32_t relocsSize = curRelocBlock->sizeOfBlock - sizeof(IMAGE_BASE_RELOCATION);
			uint16_t* relocs = (uint16_t*)(memoryOut + curRelocOffset + sizeof(IMAGE_BASE_RELOCATION));
			for (uint32_t i = 0; i < relocsSize / sizeof(uint16_t); i++) {
				uint16_t reloc = relocs[i];
				if (!reloc)
					continue;

				int offset = reloc & 0xFFF;
				auto target = GETM(uintptr_t, curRelocBlock->virtualAddress + offset);

				// Relocate
				*target += newBase - optionalHeader->imageBase;

				if (*target < newBase || *target > newBase + sizeOut)
					RS_ERR_CLOSE(ERR_PREFIX << "Relocation at 0x" << std::hex << curRelocBlock->virtualAddress << " goes outside image: 0x" << *target);

				relocationsDone++;
			}

			curRelocOffset += curRelocBlock->sizeOfBlock;
			curRelocBlock = GETM(IMAGE_BASE_RELOCATION, curRelocOffset);
		}
	}
#undef GETM
#undef READM

	RS_LOG(" > Done! Base address: " << (void*)memoryOut << ", sections count: " << fileHeader->numberOfSections << ", relocations: " << relocationsDone);
}

RS_NS_END