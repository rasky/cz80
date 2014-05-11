#include <assert.h>
#include <memory.h>
#include <stdio.h>
#include <queue>
#include <string>
#include <fstream>
#include <tuple>
#include "z80.h"

namespace Test {
    std::queue<std::tuple<u16,u8>> reads;
    std::queue<std::tuple<u16,u8>> writes;
    std::queue<std::tuple<u16,u8>> outs;
    std::queue<std::tuple<u16,u8>> ins;
}

namespace Z80Bus {

    u8 MEM[0x10000];

    u8 Rd(u16 Addr) {
        u8 val = MEM[Addr];
        Test::reads.push(std::make_pair(Addr, val));
        return val;
    }
    void Wr(u16 Addr, u8 val) {
        Test::writes.push(std::make_pair(Addr, val));
        MEM[Addr] = val;
    }
    u8 In(u16 Addr) {
        u8 val = 0xFF;
        Test::ins.push(std::make_pair(Addr, val));
        return val;
    }
    void Out(u16 Addr, u8 val) {
        Test::outs.push(std::make_pair(Addr, val));
    }
    u8 IRQVector() {
        return 0xFA;
    }

    void exc_unimplemented(u8 op) {
        fprintf(stderr, "ERROR: unimplemented exc: PC=%04x, op=%02x\n",
            (u16)::Z80::reg(::Z80::REG_PC), op);
        assert(0);
    }

}

namespace Marat {
    extern "C" {
        #include "Z80-Marat/Z80.h"
    }

    Z80 cpu;

    void reset(void)
    {
        ResetZ80(&cpu);
    }

    void WrZ80(word Addr, byte Value) {
        auto t = Test::writes.front();
        if (std::get<0>(t) != Addr || std::get<1>(t) != Value) {
            fprintf(stderr, "ERROR: mem write does not match: PC=%04x, [%04x]<-%02x  Marat[%04x]<-%02x\n",
                (u16)::Z80::reg(::Z80::REG_PC), std::get<0>(t), std::get<1>(t), Addr, Value);
            assert(0);
        }
        Test::writes.pop();
    }
    byte RdZ80(word Addr) {
        auto t = Test::reads.front();
        if (std::get<0>(t) != Addr) {
            fprintf(stderr, "ERROR: mem read does not match: PC=%04x, [%04x]->%02x  Marat[%04x]\n",
                (u16)::Z80::reg(::Z80::REG_PC), std::get<0>(t), std::get<1>(t), Addr);
            assert(0);
        }
        u8 val = std::get<1>(t);
        Test::reads.pop();
        return val;
    }
    byte InZ80(word Addr) {
        auto t = Test::ins.front();
        if (std::get<0>(t) != Addr) {
            fprintf(stderr, "ERROR: mem read does not match: PC=%04x, [%04x]->%02x  Marat[%04x]\n",
                (u16)::Z80::reg(::Z80::REG_PC), std::get<0>(t), std::get<1>(t), Addr);
            assert(0);
        }
        u8 val = std::get<1>(t);
        Test::ins.pop();
        return val;
    }
    void OutZ80(word Addr, byte Value) {
        auto t = Test::outs.front();
        if (std::get<0>(t) != Addr || std::get<1>(t) != Value) {
            fprintf(stderr, "ERROR: mem write does not match: PC=%04x, [%04x]<-%02x  Marat[%04x]<-%02x\n",
                (u16)::Z80::reg(::Z80::REG_PC), std::get<0>(t), std::get<1>(t), Addr, Value);
            assert(0);
        }
        Test::outs.pop();
    }
    void PatchZ80(Z80*) {}
}

namespace Test {
    using namespace Z80;

    void assertequal(const char* regn, u16 a, u16 b)
    {
        if (a != b) {
            fprintf(stderr, "ERROR: reg does not match: PC=%04x, %s=%04x Marat-%s:%04x\n",
                (u16)::Z80::reg(REG_PC), regn, a, regn, b);
            fprintf(stderr, "REGS:  AF=%04x BC=%04x DE=%04x HL=%04x IX=%04x IY=%04x SP=%04x PC=%04x\n",
                reg(REG_AF), reg(REG_BC), reg(REG_DE), reg(REG_HL),
                reg(REG_IX), reg(REG_IY), reg(REG_SP), reg(REG_PC));
            fprintf(stderr, "MARAT: AF=%04x BC=%04x DE=%04x HL=%04x IX=%04x IY=%04x SP=%04x PC=%04x\n",
                Marat::cpu.AF.W, Marat::cpu.BC.W, Marat::cpu.DE.W, Marat::cpu.HL.W,
                Marat::cpu.IX.W, Marat::cpu.IY.W, Marat::cpu.SP.W, Marat::cpu.PC.W);
            assert(0);
        }
    }

    void sync() {
        Marat::cpu.AF.W = reg(REG_AF);
        Marat::cpu.BC.W = reg(REG_BC);
        Marat::cpu.DE.W = reg(REG_DE);
        Marat::cpu.HL.W = reg(REG_HL);
        Marat::cpu.IX.W = reg(REG_IX);
        Marat::cpu.IY.W = reg(REG_IY);
        Marat::cpu.SP.W = reg(REG_SP);
        Marat::cpu.PC.W = reg(REG_PC);
        Marat::cpu.AF1.W = reg(REG_AF1);
        Marat::cpu.BC1.W = reg(REG_BC1);
        Marat::cpu.DE1.W = reg(REG_DE1);
        Marat::cpu.HL1.W = reg(REG_HL1);
    }

    void checksync() {
        assertequal("AF", reg(REG_AF)&~0x28, Marat::cpu.AF.W);
        assertequal("BC", reg(REG_BC), Marat::cpu.BC.W);
        assertequal("DE", reg(REG_DE), Marat::cpu.DE.W);
        assertequal("HL", reg(REG_HL), Marat::cpu.HL.W);
        assertequal("IX", reg(REG_IX), Marat::cpu.IX.W);
        assertequal("IY", reg(REG_IY), Marat::cpu.IY.W);
        assertequal("SP", reg(REG_SP), Marat::cpu.SP.W);
        assertequal("PC", reg(REG_PC), Marat::cpu.PC.W);
        assertequal("AF1", reg(REG_AF1)&~0x28, Marat::cpu.AF1.W);
        assertequal("BC1", reg(REG_BC1), Marat::cpu.BC1.W);
        assertequal("DE1", reg(REG_DE1), Marat::cpu.DE1.W);
        assertequal("HL1", reg(REG_HL1), Marat::cpu.HL1.W);
    }

    void reset_test(void)
    {
        memset(Z80Bus::MEM, 0, 0x10000);
        reset();
        Marat::ResetZ80(&Marat::cpu);
    }

    void runtestopcode(std::string testdata)
    {
        memcpy(Z80Bus::MEM, testdata.data(), testdata.size());
        step();
        Marat::ExecZ80(&Marat::cpu, 1);
        checksync();
    }

    void testirq(void)
    {
        Z80::set_irq_line(true);
        step();
        Z80::set_irq_line(false);

        Marat::ExecZ80(&Marat::cpu, 1);
        Marat::IntZ80(&Marat::cpu, Z80Bus::IRQVector());
        checksync();
    }

    void run(void)
    {
        reset_test();
        std::ifstream("pacman.code", std::ios::binary).read((char*)Z80Bus::MEM, 16*1024);
        for (int i=0;i<10000;i++) {
            step();
            Marat::ExecZ80(&Marat::cpu, 1);
            checksync();
        }
        printf("triggering irq\n");
        testirq();
        printf("triggering irq done\n");
        for (int i=0;i<10000;i++) {
            printf("PC:%04x  Marat-PC:%04x\n", (u16)reg(REG_PC), Marat::cpu.PC.W);
            step();
            Marat::ExecZ80(&Marat::cpu, 1);
            checksync();
        }
        printf("Exit PC: %04x\n", (u16)reg(REG_PC));
    }
}

int main()
{
    Test::run();
}
