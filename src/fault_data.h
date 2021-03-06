#ifndef __FAULT_DATA_H
#define __FAULT_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#define AM_RAMDUMP_MAGIC_NUM0 0x48EAAD88
#define AM_RAMDUMP_MAGIC_NUM1 0xC9705737
#define AM_RAMDUMP_MAGIC_NUM2 0x0A6B8458
#define AM_RAMDUMP_MAGIC_NUM3 0xE41A9D74
#define AM_RAMDUMP_MAGIC_NUM_SIZE (4*4)
#define AM_RAMDUMP_MAGIC_NUM_START_ADDR (SRAM_BASEADDR+(768*1024)-AM_RAMDUMP_MAGIC_NUM_SIZE)

//*****************************************************************************
//
// Data structures
//
//*****************************************************************************

//
// Define a structure for local storage in am_util_faultisr_collect_data().
// Set structure alignment to 1 byte to minimize storage requirements.
//
#pragma pack(1)
typedef struct
{
	//
    // the stack pointer when HardFault_Handler() was called
    //
	volatile uint32_t u32SP;
	//
    // Stacked registers
    //
    volatile uint32_t u32R0;
    volatile uint32_t u32R1;
    volatile uint32_t u32R2;
    volatile uint32_t u32R3;
    volatile uint32_t u32R12;
    volatile uint32_t u32LR;
    volatile uint32_t u32PC;
    volatile uint32_t u32PSR;

    //
    // Other data
    //
    volatile uint32_t u32FaultAddr;
    volatile uint32_t u32BFAR;
    volatile uint32_t u32CFSR;
    volatile uint8_t  u8MMSR;
    volatile uint8_t  u8BFSR;
    volatile uint16_t u16UFSR;

} am_fault_t;

//
// Restore the default structure alignment
//
#pragma pack()

extern volatile am_fault_t sFaultData;
extern am_hal_mcuctrl_fault_t sHalFaultData;

// The input u32IsrSP is expected to be the value of the stack pointer when
// HardFault_Handler() was called.
void am_util_faultisr_collect_data(uint32_t u32IsrSP);

#ifdef __cplusplus
}
#endif

#endif /* __FAULT_DATA_H */

