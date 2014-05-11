#ifndef __Z80_H
#define __Z80_H

#include <stdint.h>

typedef int8_t s8;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

namespace Z80Bus {
    void exc_unimplemented(u8 op);
    void Wr(u16 addr, u8 value);
    u8 Rd(u16 addr);
    void Out(u16 addr, u8 value);
    u8 In(u16 addr);
}

namespace Z80 {
    enum RegName { REG_AF, REG_BC, REG_DE, REG_HL, REG_IX, REG_IY, REG_SP, REG_PC };

    void set_nmi_line(bool status);
    void set_irq_line(bool status);
    void set_busreq_line(bool status);
    void set_reset_line(bool status);

    u16& reg(RegName r);
    void reset(void);
    void step(void);
}

#endif
