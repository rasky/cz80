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
}

namespace Z80 {

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

    void exc_unimplemented(u8 op) {
        fprintf(stderr, "ERROR: unimplemented exc: PC=%04x, op=%02x\n",
            (u16)::Z80::reg(REG_PC), op);
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
    byte InZ80(word Addr) { return 0xFF; }
    void OutZ80(word Addr, byte Value) { }
    void PatchZ80(Z80*) {}
}

namespace Test {
    using namespace Z80;

    void assertequal(const char* reg, u16 a, u16 b)
    {
        if (a != b) {
            fprintf(stderr, "ERROR: reg does not match: PC=%04x, %s=%04x Marat-%s:%04x\n",
                (u16)::Z80::reg(REG_PC), reg, a, reg, b);
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
    }

    void checksync() {
        assertequal("AF", reg(REG_AF), Marat::cpu.AF.W);
        assertequal("BC", reg(REG_BC), Marat::cpu.BC.W);
        assertequal("DE", reg(REG_DE), Marat::cpu.DE.W);
        assertequal("HL", reg(REG_HL), Marat::cpu.HL.W);
        assertequal("IX", reg(REG_IX), Marat::cpu.IX.W);
        assertequal("IY", reg(REG_IY), Marat::cpu.IY.W);
        assertequal("SP", reg(REG_SP), Marat::cpu.SP.W);
        assertequal("PC", reg(REG_PC), Marat::cpu.PC.W);
    }

    void reset_test(void)
    {
        memset(MEM, 0, 0x10000);
        reset();
        Marat::ResetZ80(&Marat::cpu);
    }

    void runtestopcode(std::string testdata)
    {
        memcpy(MEM, testdata.data(), testdata.size());
        step();
        Marat::ExecZ80(&Marat::cpu, 1);
        checksync();
    }

    void run(void)
    {
        reset_test();
        std::ifstream("pacman.code", std::ios::binary).read((char*)MEM, 16*1024);
        for (int i=0;i<100;i++) {
            step();
            Marat::ExecZ80(&Marat::cpu, 1);
            checksync();
        }
    }
}


int main()
{
    Test::run();
}
