#include "peripherals.h"

// Custom LUT for W956D8MBYA HyperRAM
#define HYPERRAM_CMD_LUT_SEQ_IDX_READ 1
#define HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_MEM 0
#define HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_REG 2

const uint32_t FLEXSPI_RAM_LUT[FLEXSPI_RAM_LUT_LENGTH] = {
	// note 1: The RAM is 8 Megabytes and one word contains 2 bytes. 
	//  Therefore the address is 22 bits long.
	//  To split upper and lower column address (see W956D8MBYA datasheet)
	//  we assign 3 Bits as column address (in FlexSPI) and 19 bits as row address.
	//  Also refer to HyperRamBitAssignment.ods

	//? note 2: The W956D8MBYA starts counting the read-latency (dummy cycles) at transmission
	//?  of the 4th byte and not at the transmission of the last byte. Therefore the dummy cycle
	//?  count is calculated in a diffrent way as FlexSPI expects it. This means we can't use
	//?  dynamic dummy cycles/initial latency and always have to use the worst case scenario.
	//?  This is 5 * 2 full clock cycles for 133MHz (so 2*10 for DDR), for this the RAM has to be 
	//?  configured though, as the default value expected by the RAM is 7 * 2.
	//?  Due to the offset between RAM and FlexSPI calculation we have to subtract 5 (DDR) dummy cycles.
	//?  As the RAM starts writingdata to the line (when reading from RAM) at the rising SCK edge
	//?  we have to add 1 (DDR) dummy cycle to the read command.


  // Write to memory in DDR
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_MEM] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, 			kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR, 	kFLEXSPI_8PAD, 24),
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_MEM + 1] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, 			kFLEXSPI_8PAD, 0x00, kFLEXSPI_Command_CADDR_DDR, 	kFLEXSPI_8PAD, 8),
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_MEM + 2] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, (7*4-2), 	 kFLEXSPI_Command_READ_DDR, 	kFLEXSPI_8PAD, 0x02),

  
  // Read from memory+reg in DDR
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_READ] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, 			kFLEXSPI_8PAD, 0xA0, kFLEXSPI_Command_RADDR_DDR, 	kFLEXSPI_8PAD, 24),
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_READ + 1] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, 			kFLEXSPI_8PAD, 0x00, kFLEXSPI_Command_CADDR_DDR, 	kFLEXSPI_8PAD, 8),
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_READ + 2] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, (7*4-1), 	 kFLEXSPI_Command_READ_DDR, 	kFLEXSPI_8PAD, 0x04),

  // Write to register in DDR
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_REG] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, 			kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR, 	kFLEXSPI_8PAD, 24),
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_REG + 1] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, 			kFLEXSPI_8PAD, 0x00, kFLEXSPI_Command_CADDR_DDR, 	kFLEXSPI_8PAD, 8),
  [4 * HYPERRAM_CMD_LUT_SEQ_IDX_WRITE_REG + 2] =
  FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR, 		kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP, 		kFLEXSPI_8PAD, 0),
};