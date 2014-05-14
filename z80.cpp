#include "z80.h"
#include <assert.h>
#include <algorithm>
#include <string>

template <unsigned bitno, unsigned nbits=1, typename T=u8>
struct RegBit {
    T data;
    enum { mask = (1u << nbits) - 1u };
    template<typename T2>
    RegBit& operator=(T2 val) {
        data = (data & ~(mask << bitno)) | ((nbits > 1 ? val : !!val) << bitno);
        return *this;
    }
    operator unsigned() const { return (data >> bitno) & mask; }
    unsigned operator &(unsigned val) const { return (unsigned)*this & val; }
    RegBit& operator &=(unsigned val) { data &= (val & mask) << bitno; return *this; }
    RegBit& operator++ ()     { return *this = *this + 1; }
    unsigned operator++ (int) { unsigned r = *this; ++*this; return r; }
};

namespace Z80Bus {
    void Wr(u16 addr, u8 value);
    u8 Rd(u16 addr);
    void Out(u16 addr, u8 value);
    u8 In(u16 addr);
    u8 IRQVector();
    void exc_unimplemented(u8 op);
}

namespace Z80 {
    using namespace std;
    using namespace Z80Bus;

    typedef union Reg_ {
        u16 W;
        struct { u8 L,H; };
        RegBit<0> CF; RegBit<1> NF; RegBit<2> VF; RegBit<2> PF; RegBit<3> XF;
        RegBit<4> HF; RegBit<5> YF; RegBit<6> ZF; RegBit<7> SF;
        operator u16() { return W; }
        Reg_& operator=(u16 val) { W=val; return *this; }
        Reg_& operator++() { return *this = *this+1; }
        u16 operator++(int) { u16 r = *this; ++*this; return r; }
        Reg_& operator--() { return *this = *this-1; }
        u16 operator--(int) { u16 r = *this; --*this; return r; }
    } Reg;

    #define LEVEL_TRIGGERED      0
    #define UP_EDGE_TRIGGERED    1
    #define DOWN_EDGE_TRIGGERED  2

    template <int trigger>
    struct Line {
        bool status; bool triggered;
        Line& operator=(bool v) { if (trigger != LEVEL_TRIGGERED && (status^v)) triggered=true; status=v; return *this; }
        operator bool() { bool ret=(trigger == LEVEL_TRIGGERED ? status : triggered); triggered = false; return ret; }
    };

    Line<LEVEL_TRIGGERED> line_irq, line_busreq, line_reset;
    Line<UP_EDGE_TRIGGERED> line_nmi;
    u64 CLK; bool ei_delay, halted;
    Reg AF, BC, DE, HL, IX, IY, SP, PC, AF1, BC1, DE1, HL1; u8 R, I, IFF1, IFF2, IM;
    auto &A = AF.H; auto &F = AF.L;
    auto &B = BC.H; auto &C = BC.L;
    auto &D = DE.H; auto &E = DE.L;
    auto &H = HL.H; auto &L = HL.L;

    u8*  const regop[8] = {&B,&C,&D,&E,&H,&L,nullptr,&A};
    Reg* const ppairop[4] = { &BC, &DE, &HL, &SP };
    Reg* const qpairop[4] = { &BC, &DE, &HL, &AF };
    u8   const flagmask[4] = { (1<<6), (1<<0), (1<<2), (1<<7) };

    #define ft256(t)    ft256a(t,0) ft256a(t,1) ft256a(t,2) ft256a(t,3)
    #define ft256a(t,n) ft256b(t,(n)*4+0) ft256b(t,(n)*4+1) ft256b(t,(n)*4+2) ft256b(t,(n)*4+3)
    #define ft256b(t,n) ft256c(t,(n)*4+0) ft256c(t,(n)*4+1) ft256c(t,(n)*4+2) ft256c(t,(n)*4+3)
    #define ft256c(t,n) ft256d(t,(n)*4+0) ft256d(t,(n)*4+1) ft256d(t,(n)*4+2) ft256d(t,(n)*4+3)
    #define ft256d(t,n) t(n)

    template <u8 val>
    u8 CalcZS() {
        Reg ret = {0};
        ret.ZF=(val?0:1); ret.SF=val>>7;
        ret.YF=val>>5; ret.XF=val>>3;
        return ret;
    }
    template <u8 val>
    u8 CalcPZS() {
        Reg ret = { CalcZS<val>() };
        ret.PF = !__builtin_parity(val);
        return ret;
    }
    #define tzs(n) CalcZS<n>(),
    const u8 ZSTable[256] = { ft256(tzs) };
    #define tpzs(n) CalcPZS<n>(),
    const u8 PZSTable[256] = { ft256(tpzs) };

#if 0
    #define xxx(xs,idx) \
        if (xs[idx] == '0' && (op>>7)) match=false; \
        if (xs[idx] == '1' && !(op>>7)) match=false; \
        if (xs[idx] == 'r') r_ = (r_<<1) | (op>>7); \
        if (xs[idx] == 's') s_ = (s_<<1) | (op>>7); \
        if (xs[idx] == 'p') p_ = (p_<<1) | (op>>7); \
        if (xs[idx] == 'q') q_ = (q_<<1) | (op>>7); \
        if (xs[idx] == 'f') f = (f<<1) | (op>>7); \
        if (xs[idx] == 'g') g = (g<<1) | (op>>7); \
        if (xs[idx] == 'm') m = (m<<1) | (op>>7); \
        op<<=1;
    #define xa(ch) \
        if (ch == 'n') n=op; \
        if (ch == 'N') n=(op<<8)|n; \
        if (ch == 'd') d=op;
    #define xx(xs,n)  \
        xxx(xs,n+0) xxx(xs,n+1) xxx(xs,n+2) xxx(xs,n+3) \
        xxx(xs,n+4) xxx(xs,n+5) xxx(xs,n+6) xxx(xs,n+7)
    #define xflag(xsf) \
        if (xsf == '@') F=PZSTable[A]; \
        if (xsf == 'n') AF.NF = 0; \
        if (xsf == 'N') AF.NF = 1; \
        if (xsf == 'h') AF.HF = 0; \
        if (xsf == 'H') AF.HF = 1;
    #define xend(xs) \
        if (match) { xflag(xsstring[0]); xflag(xsstring[1]); xflag(xsstring[2]); return; }
    #define xstart() \
        u8 op,r_,s_,p_,q_,d,f,g,m,_; u16 n; bool match=false; const char* xsstring;
    #define x(xs, cyc) \
        xend(xs) \
        r_=s_=p_=q_=d=n=f=g=m=_=0; xsstring=xs; match=true; op=op_; \
        xx(xsstring, 0)    \
        if (match && xsstring[ 8] == '.') { op=Rd(PC.W++); xa(xsstring[9]) }  \
        if (match && xsstring[ 8] == ':' && prefix) { op=Rd(PC.W++); xa(xsstring[9]) }  \
        if (match && xsstring[10] == '.') { op=Rd(PC.W++); xa(xsstring[11]) }  \
        if (match && xsstring[10] == ':' && prefix) { op=Rd(PC.W++); xa(xsstring[11]) }  \
        if (match) CLK+=cyc,xsstring+=13; \
        if (match)
    #define fallback() \
        if (m) return; \
        if (1)
    #define r (*regop[r_])
    #define s (*regop[s_])
    #define p (*ppairop[p_])
    #define q (*qpairop[q_])
#else
    #define INLINE  __attribute__((always_inline))

    #define xxx(xs,idx) \
        if      (xs[idx] == '0') { if (op>>7) { match=false; return; } } \
        else if (xs[idx] == '1') { if (!(op>>7)) { match=false; return; } } \
        else if (xs[idx] == 'r') r_ = (r_<<1) | (op>>7); \
        else if (xs[idx] == 's') s_ = (s_<<1) | (op>>7); \
        else if (xs[idx] == 'p') p_ = (p_<<1) | (op>>7); \
        else if (xs[idx] == 'q') q_ = (q_<<1) | (op>>7); \
        else if (xs[idx] == 'f') f = (f<<1) | (op>>7); \
        else if (xs[idx] == 'g') g = (g<<1) | (op>>7); \
        else if (xs[idx] == 'm') m = (m<<1) | (op>>7); \
        op<<=1;
    #define xa(ch) \
        if      (ch == 'n') n=op; \
        else if (ch == 'N') n=(op<<8)|n; \
        else if (ch == 'd') d=op;
    #define xflag(xsf) \
        if      (xsf == '@') F=PZSTable[A]; \
        else if (xsf == 'k') AF.CF = cf; \
        else if (xsf == 'w') AF.VF = vf; \
        else if (xsf == 'n') AF.NF = 0; \
        else if (xsf == 'N') AF.NF = 1; \
        else if (xsf == 'h') AF.HF = 0; \
        else if (xsf == 'H') AF.HF = 1;

    #define NextOp() (CLK+=1, Rd(PC.W++))

    struct XDecoder {
        u8 r_,s_,p_,q_,d,f,g,m,_; bool match; u16 n; const char *xsflag;
        bool cf, vf;

        XDecoder() { match=false; }
        XDecoder(u8 prefix, u8 op, const char* xs, int cyc) INLINE {
            r_=s_=p_=q_=d=n=f=g=m=_=0; match=true;
            xxx(xs, 0) xxx(xs, 1) xxx(xs, 2) xxx(xs, 3)
            xxx(xs, 4) xxx(xs, 5) xxx(xs, 6) xxx(xs, 7)
            if (xs[ 8] == '.')           { op=NextOp(); xa(xs[ 9]) }
            if (xs[ 8] == ':' && prefix) { op=NextOp(); xa(xs[ 9]) }
            if (xs[10] == '.')           { op=NextOp(); xa(xs[11]) }
            if (xs[10] == ':' && prefix) { op=NextOp(); xa(xs[11]) }
            CLK+=cyc; cf=AF.CF; vf=AF.VF;
            xsflag = xs+13;
            match=true;
        }

        ~XDecoder() INLINE {
            if (!match) return;
            if (xsflag[0] != ' ') { xflag(xsflag[0]); xflag(xsflag[1]); xflag(xsflag[2]); }
        }

        operator bool() INLINE { return match; }
    };

    #define xstart()   if (0);
    #define x(xs, cyc) else if (XDecoder x = XDecoder(prefix, op_, xs, cyc))
    #define fallback() else

    #define _ x._
    #define f x.f
    #define g x.g
    #define d x.d
    #define m x.m
    #define n x.n
    #define r (*regop[x.r_])
    #define s (*regop[x.s_])
    #define p (*ppairop[x.p_])
    #define q (*qpairop[x.q_])
#endif

    inline void Io8(bool read, u16 addr, u8 &val) {
        if (read) val = In(addr);
        else Out(addr, val);
    }

    inline void Mem8(bool read, u16 addr, u8 &val) {
        if (read) val = Rd(addr);
        else Wr(addr, val);
    }
    inline void Mem16(bool read, u16 addr, u16 &val) {
        if (read) { val = Rd(addr); val |= Rd(addr+1)<<8; }
        else { Wr(addr, val&0xFF); Wr(addr+1, val>>8); }
    }

    template <template <u8 op> class INS, u8 prefix=0>
    void OpCall() {
    #if 1
        #define ftins(n) &INS<n>::template exec<prefix>,
        static void (* const table[256])() = {
            ft256(ftins)
        };
        table[NextOp()]();
    #else
        switch (NextOp()) {
            #define ftins(n) case n: return INS<n>::exec();
            ft256(ftins)
        }
    #endif
    }

    u8 ADC(u8 s1, u8 s2) {
        u8 cf = AF.CF;
        u8 res = s1+s2;
        F = ZSTable[res+cf];
        AF.CF = bool(s1>res);
        AF.VF = bool(~(s1^s2)&(s1^res)&0x80);
        AF.HF = bool((s1&0xf) > (res&0xf));
        return res+cf;
    }

    u8 SBC(u8 s1, u8 s2) {
        u8 cf = AF.CF;
        u8 res = s1-s2;
        F = ZSTable[res-cf];
        AF.NF = 1;
        AF.CF = bool(s1<res);
        AF.VF = bool((s1^s2)&(s1^res)&0x80);
        AF.HF = bool((s1&0xf) < (res&0xf));
        return res-cf;
    }

    void LD() {
        u8 tmp = Rd(HL);
        Wr(DE, tmp);
        AF.YF = bool((tmp + A) & 0x2);
        AF.XF = bool((tmp + A) & 0x8);
        AF.PF = bool(BC != 0);
    }

    template <u8 op_> struct Ins;

    template <u8 op_> struct InsCB {
        template <u8 prefix=0>
        static void exec() {
            u8 doff = prefix?NextOp():0; u8 tmp; bool wb=true;
            auto& HL = (prefix == 0xDD) ? Z80::IX : (
                       (prefix == 0xFD) ? Z80::IY : Z80::HL);
            u8*  const regop[8] = {&B,&C,&D,&E,&H,&L,&tmp,&A};

            if (prefix || (op_ & 7) == 6) { *regop[op_&7] = Rd(HL+doff); CLK++; }

            xstart()
            x("00000rrr    |hn ", 0) { f=r>>7; r=(r<<1)|(r>>7);     F = PZSTable[r]; AF.CF=f; } // RLC
            x("00001rrr    |hn ", 0) { f=r&1;  r=(r>>1)|(r<<7);     F = PZSTable[r]; AF.CF=f; } // RRC
            x("00010rrr    |hn ", 0) { f=r>>7; r=(r<<1)|AF.CF;      F = PZSTable[r]; AF.CF=f; } // RL
            x("00011rrr    |hn ", 0) { f=r&1;  r=(r>>1)|(AF.CF<<7); F = PZSTable[r]; AF.CF=f; } // RR
            x("00100rrr    |hn ", 0) { f=r>>7; r=(r<<1)|(r&1);      F = PZSTable[r]; AF.CF=f; } // SLA
            x("00101rrr    |hn ", 0) { f=r&1;  r=(r>>1)|(r&0x80);   F = PZSTable[r]; AF.CF=f; } // SRA
            x("00110rrr    |hn ", 0) { f=r>>7; r=(r<<1);            F = PZSTable[r]; AF.CF=f; } // SLL
            x("00111rrr    |hn ", 0) { f=r&1;  r=(r>>1);            F = PZSTable[r]; AF.CF=f; } // SRL
            x("01fffrrr    |Hnk", 0) { g = r & (1<<f); F = PZSTable[g]; wb=false; }
            x("10fffrrr    |   ", 0) { r &= ~(1<<f); }
            x("11fffrrr    |   ", 0) { r |=  (1<<f); }
            fallback()           { exc_unimplemented(op_); }

            if (wb && (prefix || (op_ & 7) == 6)) { Wr(HL+doff, *regop[op_&7]); CLK++; }
        }
    };

    template <u8 op_> struct InsDD {
        template <u8 prefix=0>
        static void exec() {
            xstart()
            x("01000111    |   ", 0) { I=A; }
            x("01001111    |   ", 0) { R=A; }
            x("01010111    |hnk", 0) { A=I; F=ZSTable[A]; AF.PF=IFF2; }
            x("0101A111    |hnk", 0) { A=R; F=ZSTable[A]; AF.PF=IFF2; }
            fallback()           { Ins<op_>::template exec<0xDD>(); }
        }
    };

    template <u8 op_> struct InsED {
        template <u8 prefix=0>
        static void exec() {
            xstart()
            x("01000100    |   ", 2) { AF.CF=0; A=SBC(0,A); }
            x("01ppm011    |   ", 6) { Mem16(m, n, p.W); }
            x("11rrr00m    |   ", 3) { Io8(!m, BC, r); /* FLAG */ }
            x("101gf010    |   ", 4) { Wr(HL, In(BC));  HL.W-=f*2-1; if (--B && g) { PC.W-=2; CLK++; } /* FLAG */ }
            x("101gf011    |   ", 4) { Out(BC, Rd(HL)); HL.W-=f*2-1; if (--B && g) { PC.W-=2; CLK++; } /* FLAG */ }
            x("01ppg010    |!  ", 4) { HL.L=(g?SBC:ADC)(HL.L, p.L); HL.H=(g?SBC:ADC)(HL.H, p.H); }
            x("01000110    |   ", 2) { IM=0; }
            x("01010110    |   ", 2) { IM=1; }
            x("01011110    |   ", 2) { IM=2; }
            x("01001101    |   ", 4) { PC.L=Rd(SP++); PC.H=Rd(SP++); }
            x("01000101    |   ", 4) { PC.L=Rd(SP++); PC.H=Rd(SP++); IFF1=IFF2; }
            x("01000111    |   ", 2) { I=A; }
            x("01010111    |   ", 2) { A=I; /* FLAG */}
            x("01101111    |   ", 5) { g=f=Rd(HL); f = (f<<4)|(A&0xF); A = (A&0xF0)|(g>>4); Wr(HL, f); }
            x("101gf000    |hn ", 4) { BC--; LD(); DE.W-=f*2-1; HL.W-=f*2-1; if (BC && g) { PC.W-=2; CLK++; } } // LD*
            fallback()           { exc_unimplemented(op_); }
        }
    };

    template <u8 op_> struct InsFD {
        template <u8 prefix=0>
        static void exec() {
            return Ins<op_>::template exec<0xFD>();
        }
    };

    template <u8 op_> struct Ins {
        template <u8 prefix=0>
        static void exec() {
            auto& HL = (prefix == 0xDD) ? Z80::IX : (
                       (prefix == 0xFD) ? Z80::IY : Z80::HL);
            Reg* const ppairop[4] = { &BC, &DE, &HL, &SP };
            Reg* const qpairop[4] = { &BC, &DE, &HL, &AF };
            u8*  const regop[8] = {&B,&C,&D,&E,&HL.H,&HL.L,nullptr,&A};

            xstart()
            x("00000000    |   ", 0) {}
            x("01110110    |   ", 0) { PC.W -= 1; halted=true; }
            x("11110011    |   ", 0) { IFF1=IFF2=0; }
            x("11111011    |   ", 0) { IFF1=IFF2=1; ei_delay=true; }

            x("0000m010    |   ", 1) { Mem8(m, BC, A); }
            x("0001m010    |   ", 1) { Mem8(m, DE, A); }
            x("0010m010.n.N|   ", 2) { Mem16(m, n, HL.W); }
            x("00pp0001.n.N|   ", 0) { p=n; }
            x("00001010    |   ", 1) { A=Rd(HL); }
            x("0011m010.n.N|   ", 1) { Mem8(m, n, A); }
            x("00110110:d.n|   ", 1) { Wr(HL+d, n); }
            x("00rrr110.n  |   ", 0) { r=n; }
            x("01rrr110:d  |   ", 1) { r=Rd(HL+d); }
            x("01110rrr:d  |   ", 1) { Wr(HL+d, *Z80::regop[x.r_]); } // TEST!!!
            x("01rrrsss    |   ", 0) { r=s; }

            x("1101m011.n  |   ", 1) { Io8(m, ((A<<8)|n), A); }

            x("00101111    |*HN", 0) { A=~A; }
            x("00110111    |*hn", 0) { AF.CF=1; }
            x("00111111    |*n ", 0) { AF.HF=AF.CF; AF.CF=~AF.CF; }

            x("00110100:d  |!k ", 0) { AF.CF=0; Wr(HL, ADC(Rd(HL+d),1)); }
            x("00rrr10g    |!k ", 0) { AF.CF=0; r=(g?SBC:ADC)(r,1); }
            x("00pp1001    |!w ", 2) { AF.CF=0; HL.L=ADC(HL.L, p.L); HL.H=ADC(HL.H, p.H); AF.ZF=!HL.W; }
            x("00ppf011    |   ", 0) { p.W -= f*2-1; }

            x("100gf110:d  |!  ", 1) { AF.CF&=f; A=(g?SBC:ADC)(A,Rd(HL+d)); }
            x("100gfrrr    |!  ", 0) { AF.CF&=f; A=(g?SBC:ADC)(A,r); }
            x("110gf110.n  |!  ", 0) { AF.CF&=f; A=(g?SBC:ADC)(A,n); }
            x("10111110:d  |!  ", 1) { AF.CF=0; (void)SBC(A,Rd(HL+d)); }
            x("10111rrr    |!  ", 0) { AF.CF=0; (void)SBC(A,r); }
            x("11111110.n  |!  ", 0) { AF.CF=0; (void)SBC(A,n); }

            x("10100110:d  |@H ", 1) { A&=Rd(HL+d); }
            x("10100rrr    |@H ", 0) { A&=r; }
            x("11100110.n  |@H ", 0) { A&=n; }
            x("10110110:d  |@  ", 1) { A|=Rd(HL+d); }
            x("10110rrr    |@  ", 0) { A|=r; }
            x("11110110.n  |@  ", 0) { A|=n; }
            x("10101110:d  |@  ", 1) { A^=Rd(HL+d); }
            x("10101rrr    |@  ", 0) { A^=r; } /* XOR reg */
            x("11101110.n  |@  ", 0) { A^=n; }

            x("00000111    |hn ", 1) { A=(A>>7)|(A<<1);     AF.CF=A&1;      }
            x("00001111    |hn ", 1) { A=(A<<7)|(A>>1);     AF.CF=A&1;      }
            x("00010111    |hn ", 1) { _=(A<<1)|AF.CF;      AF.CF=A&1; A=_; }
            x("00011111    |hn ", 1) { _=(A>>1)|(AF.CF<<7); AF.CF=A&1; A=_; }

            x("11qq0101    |   ", 2) { Wr(--SP, q.H); Wr(--SP, q.L); }
            x("11qq0001    |   ", 2) { q.L = Rd(SP++); q.H = Rd(SP++); }
            x("11111001    |   ", 0) { SP = HL; }
            x("11101011    |   ", 0) { swap(DE.W, Z80::HL.W); }
            x("00001000    |   ", 0) { swap(AF.W, AF1.W); }
            x("11011001    |   ", 0) { swap(BC.W, BC1.W); swap(DE.W, DE1.W); swap(Z80::HL.W, HL1.W); }
            x("11100011    |   ", 4) { Mem16(true, SP, n); Mem16(false, SP, HL.W); HL=n; }

            x("11000011.n.N|   ", 0) { PC = n; }
            x("11ffg010    |   ", 2) { if ((g?F:~F)&flagmask[f]) CLK+=2,Mem16(true, PC, PC.W); else PC.W+=2; }
            x("00011000.n  |   ", 1) { PC.W += (s8)n; }
            x("00111000    |   ", 1) { if (AF.CF) { PC.W += (s8)Rd(PC); CLK+=1; } PC.W+=1; }
            x("00110000    |   ", 1) { if (!AF.CF) { PC.W += (s8)Rd(PC); CLK+=1; } PC.W+=1; }
            x("00101000    |   ", 1) { if (AF.ZF) { PC.W += (s8)Rd(PC); CLK+=1; } PC.W+=1; }
            x("00100000    |   ", 1) { if (!AF.ZF) { PC.W += (s8)Rd(PC); CLK+=1; } PC.W+=1; }
            x("11101001    |   ", 1) { PC = HL; }
            x("00010000    |   ", 1) { if (--B) { PC.W +=(s8)Rd(PC); CLK+=1; } PC.W+=1; }

            x("11ffg100    |   ", 2) { PC.W+=2; if ((g?F:~F)&flagmask[f]) { Mem16(true, PC.W-2, n); Wr(--SP, PC.H); Wr(--SP, PC.L); CLK+=2; PC.W=n; } }
            x("11001101.n.N|   ", 2) { Wr(--SP, PC.H); Wr(--SP, PC.L); PC.W=n; }
            x("11ffg000    |   ", 0) { if ((g?F:~F)&flagmask[f]) { PC.L=Rd(SP++); PC.H=Rd(SP++); CLK+=2; } }
            x("11001001    |   ", 2) { PC.L=Rd(SP++); PC.H=Rd(SP++); }
            x("11fff111    |   ", 2) { Wr(--SP, PC.H); Wr(--SP, PC.L); PC.W=f*8; }

            x("11001011    |   ", 0) { OpCall<InsCB,prefix>(); }
            x("11011101    |   ", 0) { OpCall<InsDD>(); }
            x("11101101    |   ", 0) { OpCall<InsED>(); }
            x("11111101    |   ", 0) { OpCall<InsFD>(); }

            fallback()               { exc_unimplemented(op_); }
        }
    };

    void reset(void) {
        AF=AF1= BC=BC1= DE=DE1= HL=HL1= IX= IY= SP= PC= R= I= IFF1= IFF2= IM=0;
        SP = 0xF000;
        ei_delay = halted = false;
    }

    void nmi(void) {
        IFF1 = 0;
        if (halted) { PC.W++; halted = false; }
        Wr(--SP, PC.H); Wr(--SP, PC.L);
        PC.W = 0x66;
    }

    void irq(void) {
        if (!IFF1) return;
        IFF1 = IFF2 = 0;
        if (halted) { PC.W++; halted = false; }
        Wr(--SP, PC.H); Wr(--SP, PC.L);
        switch (IM) {
            case 0: assert(!"not implemented: IM=0"); break;
            case 1: PC.W = 0x38; break;
            case 2: Mem16(true, (I<<8)|IRQVector(), PC.W); break;
        }
    }

    void set_nmi_line(bool status)    { line_nmi = status; }
    void set_irq_line(bool status)    { line_irq = status; }
    void set_busreq_line(bool status) { line_busreq = status; }
    void set_reset_line(bool status)  { line_reset = status; }

    void step() {
        OpCall<Z80::Ins>();
        if (line_irq && !ei_delay)
            irq();
        if (line_nmi)
            nmi();
        ei_delay = false;
    }

    u16& reg(RegName reg) {
        switch (reg) {
            case REG_AF: return AF.W;
            case REG_BC: return BC.W;
            case REG_DE: return DE.W;
            case REG_HL: return HL.W;
            case REG_IX: return IX.W;
            case REG_IY: return IY.W;
            case REG_SP: return SP.W;
            case REG_PC: return PC.W;
            case REG_AF1: return AF1.W;
            case REG_BC1: return BC1.W;
            case REG_DE1: return DE1.W;
            case REG_HL1: return HL1.W;
        }
    }
}
