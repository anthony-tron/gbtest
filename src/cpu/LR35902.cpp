#include <cassert>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "LR35902.h"

gbtest::LR35902::LR35902(Bus& bus)
        : m_bus(bus)
        , m_interruptController(bus)
        , m_cyclesToWait(0)
        , m_halted(false)
        , m_stopped(false)
        , m_tickCounter(0)
{

}

void gbtest::LR35902::setRegisters(const LR35902Registers& registers)
{
    m_registers = registers;
}

const gbtest::LR35902Registers& gbtest::LR35902::getRegisters() const
{
    return m_registers;
}

const gbtest::InterruptController& gbtest::LR35902::getInterruptController() const
{
    return m_interruptController;
}

gbtest::InterruptController& gbtest::LR35902::getInterruptController()
{
    return m_interruptController;
}

void gbtest::LR35902::setHalted(bool halted)
{
    m_halted = halted;
}

const bool& gbtest::LR35902::isHalted() const
{
    return m_halted;
}

void gbtest::LR35902::setStopped(bool stopped)
{
    m_stopped = stopped;
}

const bool& gbtest::LR35902::isStopped() const
{
    return m_stopped;
}

const uint8_t& gbtest::LR35902::getCyclesToWaste() const
{
    return m_cyclesToWait;
}

const unsigned& gbtest::LR35902::getTickCounter() const
{
    return m_tickCounter;
}

void gbtest::LR35902::tick()
{
    // Tick the interrupt controller
    m_interruptController.tick();

    if (m_cyclesToWait == 0) {
        // Handle interrupts before fetching the instruction
        handleInterrupt();

        // Execute current instruction
        const uint8_t opcode = fetch();
        try {
            lookUp(opcode);
        }
        catch (const std::runtime_error& e) {
            std::cerr << std::uppercase << std::hex
                      << "PC = 0x" << m_registers.pc << "; Opcode = 0x" << (int) opcode << std::endl
                      << "Caught exception: " << e.what() << std::endl;
        }

        // Handle delayed interrupt enable
        m_interruptController.handleDelayedInterrupt();
    }

    ++m_tickCounter;
    --m_cyclesToWait;
}

void gbtest::LR35902::step()
{
    if (m_cyclesToWait > 0) {
        // Simulate the waste of all cycles
        m_tickCounter += m_cyclesToWait - 1;
        m_cyclesToWait = 0;
    }

    // Execute the instruction
    tick();
}

uint8_t gbtest::LR35902::fetch()
{
    return m_bus.read(m_registers.pc++, gbtest::BusRequestSource::CPU);
}

void gbtest::LR35902::handleInterrupt()
{
    // Don't do anything if interrupts are disabled
    if (!m_interruptController.isInterruptMasterEnabled()) { return; }

    // Check if an interrupt has been requested
    const uint8_t requestedInterrupts =
            m_interruptController.getInterruptRequest() & m_interruptController.getInterruptEnable();

    // Fast exit if there are no requested interrupts
    if (requestedInterrupts == 0x00) { return; }

    // Find what interrupt is to be serviced
    uint16_t vectorAddress;
    size_t i = 0;

    while (i < 5) {
        if ((requestedInterrupts & (1 << i)) != 0) {
            vectorAddress = 0x0040 + (8 * i);
            break;
        }

        ++i;
    }

    // Return if no interrupt was requested
    if (i == 5) { return; }

    // Handle the requested interrupt
    // Start by resetting the request flag and the master enable
    m_interruptController.setInterruptRequested(static_cast<InterruptType>(1 << i), false);
    m_interruptController.setInterruptMasterEnable(false);

    // Call the interrupt vector
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = vectorAddress;

    m_cyclesToWait = 20;
}

// NOP
void gbtest::LR35902::opcode00h()
{
    m_cyclesToWait = 4;
}

// LD BC, d16
void gbtest::LR35902::opcode01h()
{
    m_registers.c = fetch();
    m_registers.b = fetch();
    m_cyclesToWait = 12;
}

// LD (BC), A
void gbtest::LR35902::opcode02h()
{
    m_bus.write(m_registers.bc, m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// INC BC
void gbtest::LR35902::opcode03h()
{
    ++m_registers.bc;

    m_cyclesToWait = 8;
}

// INC B
void gbtest::LR35902::opcode04h()
{
    INC_r8(m_registers.b);
}

// DEC B
void gbtest::LR35902::opcode05h()
{
    DEC_r8(m_registers.b);
}

// LD B, d8
void gbtest::LR35902::opcode06h()
{
    m_registers.b = fetch();
    m_cyclesToWait = 8;
}

// RLCA
void gbtest::LR35902::opcode07h()
{
    m_registers.f.c = (m_registers.a >> 7) & 0x1;

    m_registers.a = (m_registers.a << 1) | m_registers.f.c;

    m_registers.f.z = 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 4;
}

// LD (a16), SP
void gbtest::LR35902::opcode08h()
{
    uint16_t addr = fetch() | (fetch() << 8);
    m_bus.write(addr, m_registers.sp, gbtest::BusRequestSource::CPU);
    m_bus.write(addr + 1, m_registers.sp >> 8, gbtest::BusRequestSource::CPU);

    m_cyclesToWait = 20;
}

// ADD HL, BC
void gbtest::LR35902::opcode09h()
{
    ADD_HL_r16(m_registers.bc);
}

// LD A, (BC)
void gbtest::LR35902::opcode0Ah()
{
    m_registers.a = m_bus.read(m_registers.bc, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// DEC BC
void gbtest::LR35902::opcode0Bh()
{
    --m_registers.bc;

    m_cyclesToWait = 8;
}

// INC C
void gbtest::LR35902::opcode0Ch()
{
    INC_r8(m_registers.c);
}

// DEC C
void gbtest::LR35902::opcode0Dh()
{
    DEC_r8(m_registers.c);
}

// LD C, d8
void gbtest::LR35902::opcode0Eh()
{
    m_registers.c = fetch();
    m_cyclesToWait = 8;
}

// RRCA
void gbtest::LR35902::opcode0Fh()
{
    m_registers.f.c = m_registers.a & 0x1;

    m_registers.a = (m_registers.a >> 1) | (m_registers.f.c << 7);

    m_registers.f.z = 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 4;
}

// STOP
void gbtest::LR35902::opcode10h()
{
    // TODO: Implement that
    m_halted = true;
    m_stopped = true;

    m_cyclesToWait = 4;
}

// LD DE, d16
void gbtest::LR35902::opcode11h()
{
    m_registers.e = fetch();
    m_registers.d = fetch();
    m_cyclesToWait = 12;
}

// LD (DE), A
void gbtest::LR35902::opcode12h()
{
    m_bus.write(m_registers.de, m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// INC DE
void gbtest::LR35902::opcode13h()
{
    ++m_registers.de;

    m_cyclesToWait = 8;
}

// INC D
void gbtest::LR35902::opcode14h()
{
    INC_r8(m_registers.d);
}

// DEC D
void gbtest::LR35902::opcode15h()
{
    DEC_r8(m_registers.d);
}

// LD D, d8
void gbtest::LR35902::opcode16h()
{
    m_registers.d = fetch();
    m_cyclesToWait = 8;
}

// RLA
void gbtest::LR35902::opcode17h()
{
    const uint8_t newCarry = (m_registers.a >> 7) & 0x1;

    m_registers.a = (m_registers.a << 1) | m_registers.f.c;

    m_registers.f.z = 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = newCarry;

    m_cyclesToWait = 4;
}

// JR r8
void gbtest::LR35902::opcode18h()
{
    m_registers.pc += (int8_t) fetch();
    m_cyclesToWait = 12;
}

// ADD HL, DE
void gbtest::LR35902::opcode19h()
{
    ADD_HL_r16(m_registers.de);
}

// LD A, (DE)
void gbtest::LR35902::opcode1Ah()
{
    m_registers.a = m_bus.read(m_registers.de, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// DEC DE
void gbtest::LR35902::opcode1Bh()
{
    --m_registers.de;

    m_cyclesToWait = 8;
}

// INC E
void gbtest::LR35902::opcode1Ch()
{
    INC_r8(m_registers.e);
}

// DEC E
void gbtest::LR35902::opcode1Dh()
{
    DEC_r8(m_registers.e);
}

// LD E, d8
void gbtest::LR35902::opcode1Eh()
{
    m_registers.e = fetch();
    m_cyclesToWait = 8;
}

// RRA
void gbtest::LR35902::opcode1Fh()
{
    const uint8_t newCarry = m_registers.a & 0x1;

    m_registers.a = (m_registers.a >> 1) | (m_registers.f.c << 7);

    m_registers.f.z = 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = newCarry;

    m_cyclesToWait = 4;
}

// JR NZ, r8
void gbtest::LR35902::opcode20h()
{
    const auto val = (int8_t) fetch();

    if (m_registers.f.z) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc += val;
    m_cyclesToWait = 12;
}

// LD HL, d16
void gbtest::LR35902::opcode21h()
{
    m_registers.l = fetch();
    m_registers.h = fetch();
    m_cyclesToWait = 12;
}

// LD (HL+), A
void gbtest::LR35902::opcode22h()
{
    m_bus.write(m_registers.hl++, m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// INC HL
void gbtest::LR35902::opcode23h()
{
    ++m_registers.hl;

    m_cyclesToWait = 8;
}

// INC H
void gbtest::LR35902::opcode24h()
{
    INC_r8(m_registers.h);
}

// DEC H
void gbtest::LR35902::opcode25h()
{
    DEC_r8(m_registers.h);
}

// LD H, d8
void gbtest::LR35902::opcode26h()
{
    m_registers.h = fetch();
    m_cyclesToWait = 8;
}

// DAA
void gbtest::LR35902::opcode27h()
{
    if (m_registers.f.n == 0) {
        // Previous operation was an addition
        if (m_registers.f.c || m_registers.a > 0x99) {
            m_registers.a += 0x60;
            m_registers.f.c = 1;
        }

        if (m_registers.f.h || (m_registers.a & 0x0F) > 0x09) {
            m_registers.a += 0x06;
        }
    }
    else {
        // Previous operation was a subtraction
        if (m_registers.f.c) {
            m_registers.a -= 0x60;
        }

        if (m_registers.f.h) {
            m_registers.a -= 0x06;
        }
    }

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0x00);
    m_registers.f.h = 0;
}

// JR Z, r8
void gbtest::LR35902::opcode28h()
{
    const auto val = (int8_t) fetch();

    if (!m_registers.f.z) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc += val;
    m_cyclesToWait = 12;
}

// ADD HL, HL
void gbtest::LR35902::opcode29h()
{
    ADD_HL_r16(m_registers.hl);
}

// LD A, (HL+)
void gbtest::LR35902::opcode2Ah()
{
    m_registers.a = m_bus.read(m_registers.hl++, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// DEC HL
void gbtest::LR35902::opcode2Bh()
{
    --m_registers.hl;

    m_cyclesToWait = 8;
}

// INC L
void gbtest::LR35902::opcode2Ch()
{
    INC_r8(m_registers.l);
}

// DEC L
void gbtest::LR35902::opcode2Dh()
{
    DEC_r8(m_registers.l);
}

// LD L, d8
void gbtest::LR35902::opcode2Eh()
{
    m_registers.l = fetch();
    m_cyclesToWait = 8;
}

// CPL
void gbtest::LR35902::opcode2Fh()
{
    m_registers.a = ~m_registers.a;

    m_registers.f.n = 1;
    m_registers.f.h = 1;

    m_cyclesToWait = 4;
}

// JR NC, r8
void gbtest::LR35902::opcode30h()
{
    const auto val = (int8_t) fetch();

    if (m_registers.f.c) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc += val;
    m_cyclesToWait = 12;
}

// LD SP, d16
void gbtest::LR35902::opcode31h()
{
    m_registers.sp = fetch() | (fetch() << 8);
    m_cyclesToWait = 12;
}

// LD (HL-), A
void gbtest::LR35902::opcode32h()
{
    m_bus.write(m_registers.hl--, m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// INC SP
void gbtest::LR35902::opcode33h()
{
    ++m_registers.sp;

    m_cyclesToWait = 8;
}

// INC (HL)
void gbtest::LR35902::opcode34h()
{
    const uint8_t val = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU) + 1;
    m_bus.write(m_registers.hl, val, gbtest::BusRequestSource::CPU);

    m_registers.f.z = val == 0;
    m_registers.f.n = 0;
    m_registers.f.h = (val == 0x00 || val == 0x10);

    m_cyclesToWait = 12;
}

// DEC (HL)
void gbtest::LR35902::opcode35h()
{
    const uint8_t val = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU) - 1;
    m_bus.write(m_registers.hl, val, gbtest::BusRequestSource::CPU);

    m_registers.f.z = val == 0;
    m_registers.f.n = 1;
    m_registers.f.h = val == 0xF;

    m_cyclesToWait = 12;
}

// LD (HL), d8
void gbtest::LR35902::opcode36h()
{
    m_bus.write(m_registers.hl, fetch(), gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 12;
}

// SCF
void gbtest::LR35902::opcode37h()
{
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = 1;

    m_cyclesToWait = 4;
}

// JR C, r8
void gbtest::LR35902::opcode38h()
{
    const auto val = (int8_t) fetch();

    if (!m_registers.f.c) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc += val;
    m_cyclesToWait = 12;
}

// ADD HL, SP
void gbtest::LR35902::opcode39h()
{
    ADD_HL_r16(m_registers.sp);
}

// LD A, (HL-)
void gbtest::LR35902::opcode3Ah()
{
    m_registers.a = m_bus.read(m_registers.hl--, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// DEC SP
void gbtest::LR35902::opcode3Bh()
{
    --m_registers.sp;

    m_cyclesToWait = 8;
}

// INC A
void gbtest::LR35902::opcode3Ch()
{
    INC_r8(m_registers.a);
}

// DEC A
void gbtest::LR35902::opcode3Dh()
{
    DEC_r8(m_registers.a);
}

// LD A, d8
void gbtest::LR35902::opcode3Eh()
{
    m_registers.a = fetch();
    m_cyclesToWait = 8;
}

// CCF
void gbtest::LR35902::opcode3Fh()
{
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = ~m_registers.f.c;

    m_cyclesToWait = 4;
}

// LD B, B
void gbtest::LR35902::opcode40h()
{
    m_registers.b = m_registers.b;
    m_cyclesToWait = 4;
}

// LD B, C
void gbtest::LR35902::opcode41h()
{
    m_registers.b = m_registers.c;
    m_cyclesToWait = 4;
}

// LD B, D
void gbtest::LR35902::opcode42h()
{
    m_registers.b = m_registers.d;
    m_cyclesToWait = 4;
}

// LD B, E
void gbtest::LR35902::opcode43h()
{
    m_registers.b = m_registers.e;
    m_cyclesToWait = 4;
}

// LD B, H
void gbtest::LR35902::opcode44h()
{
    m_registers.b = m_registers.h;
    m_cyclesToWait = 4;
}

// LD B, L
void gbtest::LR35902::opcode45h()
{
    m_registers.b = m_registers.l;
    m_cyclesToWait = 4;
}

// LD B, (HL)
void gbtest::LR35902::opcode46h()
{
    m_registers.b = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD B, A
void gbtest::LR35902::opcode47h()
{
    m_registers.b = m_registers.a;
    m_cyclesToWait = 4;
}

// LD C, B
void gbtest::LR35902::opcode48h()
{
    m_registers.c = m_registers.b;
    m_cyclesToWait = 4;
}

// LD C, C
void gbtest::LR35902::opcode49h()
{
    m_registers.c = m_registers.c;
    m_cyclesToWait = 4;
}

// LD C, D
void gbtest::LR35902::opcode4Ah()
{
    m_registers.c = m_registers.d;
    m_cyclesToWait = 4;
}

// LD C, E
void gbtest::LR35902::opcode4Bh()
{
    m_registers.c = m_registers.e;
    m_cyclesToWait = 4;
}

// LD C, H
void gbtest::LR35902::opcode4Ch()
{
    m_registers.c = m_registers.h;
    m_cyclesToWait = 4;
}

// LD C, L
void gbtest::LR35902::opcode4Dh()
{
    m_registers.c = m_registers.l;
    m_cyclesToWait = 4;
}

// LD C, (HL)
void gbtest::LR35902::opcode4Eh()
{
    m_registers.c = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD C, A
void gbtest::LR35902::opcode4Fh()
{
    m_registers.c = m_registers.a;
    m_cyclesToWait = 4;
}

// LD D, B
void gbtest::LR35902::opcode50h()
{
    m_registers.d = m_registers.b;
    m_cyclesToWait = 4;
}

// LD D, C
void gbtest::LR35902::opcode51h()
{
    m_registers.d = m_registers.c;
    m_cyclesToWait = 4;
}

// LD D, D
void gbtest::LR35902::opcode52h()
{
    m_registers.d = m_registers.d;
    m_cyclesToWait = 4;
}

// LD D, E
void gbtest::LR35902::opcode53h()
{
    m_registers.d = m_registers.e;
    m_cyclesToWait = 4;
}

// LD D, H
void gbtest::LR35902::opcode54h()
{
    m_registers.d = m_registers.h;
    m_cyclesToWait = 4;
}

// LD D, L
void gbtest::LR35902::opcode55h()
{
    m_registers.d = m_registers.l;
    m_cyclesToWait = 4;
}

// LD D, (HL)
void gbtest::LR35902::opcode56h()
{
    m_registers.d = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD D, A
void gbtest::LR35902::opcode57h()
{
    m_registers.d = m_registers.a;
    m_cyclesToWait = 4;
}

// LD E, B
void gbtest::LR35902::opcode58h()
{
    m_registers.e = m_registers.b;
    m_cyclesToWait = 4;
}

// LD E, C
void gbtest::LR35902::opcode59h()
{
    m_registers.e = m_registers.c;
    m_cyclesToWait = 4;
}

// LD E, D
void gbtest::LR35902::opcode5Ah()
{
    m_registers.e = m_registers.d;
    m_cyclesToWait = 4;
}

// LD E, E
void gbtest::LR35902::opcode5Bh()
{
    m_registers.e = m_registers.e;
    m_cyclesToWait = 4;
}

// LD E, H
void gbtest::LR35902::opcode5Ch()
{
    m_registers.e = m_registers.h;
    m_cyclesToWait = 4;
}

// LD E, L
void gbtest::LR35902::opcode5Dh()
{
    m_registers.e = m_registers.l;
    m_cyclesToWait = 4;
}

// LD E, (HL)
void gbtest::LR35902::opcode5Eh()
{
    m_registers.e = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD E, A
void gbtest::LR35902::opcode5Fh()
{
    m_registers.e = m_registers.a;
    m_cyclesToWait = 4;
}

// LD H, B
void gbtest::LR35902::opcode60h()
{
    m_registers.h = m_registers.b;
    m_cyclesToWait = 4;
}

// LD H, C
void gbtest::LR35902::opcode61h()
{
    m_registers.h = m_registers.c;
    m_cyclesToWait = 4;
}

// LD H, D
void gbtest::LR35902::opcode62h()
{
    m_registers.h = m_registers.d;
    m_cyclesToWait = 4;
}

// LD H, E
void gbtest::LR35902::opcode63h()
{
    m_registers.h = m_registers.e;
    m_cyclesToWait = 4;
}

// LD H, H
void gbtest::LR35902::opcode64h()
{
    m_registers.h = m_registers.h;
    m_cyclesToWait = 4;
}

// LD H, L
void gbtest::LR35902::opcode65h()
{
    m_registers.h = m_registers.l;
    m_cyclesToWait = 4;
}

// LD H, (HL)
void gbtest::LR35902::opcode66h()
{
    m_registers.h = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD H, A
void gbtest::LR35902::opcode67h()
{
    m_registers.h = m_registers.a;
    m_cyclesToWait = 4;
}

// LD L, B
void gbtest::LR35902::opcode68h()
{
    m_registers.l = m_registers.b;
    m_cyclesToWait = 4;
}

// LD L, C
void gbtest::LR35902::opcode69h()
{
    m_registers.l = m_registers.c;
    m_cyclesToWait = 4;
}

// LD L, D
void gbtest::LR35902::opcode6Ah()
{
    m_registers.l = m_registers.d;
    m_cyclesToWait = 4;
}

// LD L, E
void gbtest::LR35902::opcode6Bh()
{
    m_registers.l = m_registers.e;
    m_cyclesToWait = 4;
}

// LD L, H
void gbtest::LR35902::opcode6Ch()
{
    m_registers.l = m_registers.h;
    m_cyclesToWait = 4;
}

// LD L, L
void gbtest::LR35902::opcode6Dh()
{
    m_registers.l = m_registers.l;
    m_cyclesToWait = 4;
}

// LD L, (HL)
void gbtest::LR35902::opcode6Eh()
{
    m_registers.l = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 4;
}

// LD L, A
void gbtest::LR35902::opcode6Fh()
{
    m_registers.l = m_registers.a;
    m_cyclesToWait = 4;
}

// LD (HL), B
void gbtest::LR35902::opcode70h()
{
    m_bus.write(m_registers.hl, m_registers.b, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD (HL), C
void gbtest::LR35902::opcode71h()
{
    m_bus.write(m_registers.hl, m_registers.c, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD (HL), D
void gbtest::LR35902::opcode72h()
{
    m_bus.write(m_registers.hl, m_registers.d, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD (HL), E
void gbtest::LR35902::opcode73h()
{
    m_bus.write(m_registers.hl, m_registers.e, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD (HL), H
void gbtest::LR35902::opcode74h()
{
    m_bus.write(m_registers.hl, m_registers.h, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD (HL), L
void gbtest::LR35902::opcode75h()
{
    m_bus.write(m_registers.hl, m_registers.l, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// HALT
void gbtest::LR35902::opcode76h()
{
    // TODO: Implement that
    m_halted = true;
    m_cyclesToWait = 4;
}

// LD (HL), A
void gbtest::LR35902::opcode77h()
{
    m_bus.write(m_registers.hl, m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD A, B
void gbtest::LR35902::opcode78h()
{
    m_registers.a = m_registers.b;
    m_cyclesToWait = 4;
}

// LD A, C
void gbtest::LR35902::opcode79h()
{
    m_registers.a = m_registers.c;
    m_cyclesToWait = 4;
}

// LD A, D
void gbtest::LR35902::opcode7Ah()
{
    m_registers.a = m_registers.d;
    m_cyclesToWait = 4;
}

// LD A, E
void gbtest::LR35902::opcode7Bh()
{
    m_registers.a = m_registers.e;
    m_cyclesToWait = 4;
}

// LD A, H
void gbtest::LR35902::opcode7Ch()
{
    m_registers.a = m_registers.h;
    m_cyclesToWait = 4;
}

// LD A, L
void gbtest::LR35902::opcode7Dh()
{
    m_registers.a = m_registers.l;
    m_cyclesToWait = 4;
}

// LD A, (HL)
void gbtest::LR35902::opcode7Eh()
{
    m_registers.a = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// LD A, A
void gbtest::LR35902::opcode7Fh()
{
    m_cyclesToWait = 4;
}

// ADD A, B
void gbtest::LR35902::opcode80h()
{
    ADD_A(m_registers.b);
}

// ADD A, C
void gbtest::LR35902::opcode81h()
{
    ADD_A(m_registers.c);
}

// ADD A, D
void gbtest::LR35902::opcode82h()
{
    ADD_A(m_registers.d);
}

// ADD A, E
void gbtest::LR35902::opcode83h()
{
    ADD_A(m_registers.e);
}

// ADD A, H
void gbtest::LR35902::opcode84h()
{
    ADD_A(m_registers.h);
}

// ADD A, L
void gbtest::LR35902::opcode85h()
{
    ADD_A(m_registers.l);
}

// ADD A, (HL)
void gbtest::LR35902::opcode86h()
{
    ADD_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// ADD A, A
void gbtest::LR35902::opcode87h()
{
    ADD_A(m_registers.a);
}

// ADC A, B
void gbtest::LR35902::opcode88h()
{
    ADC_A(m_registers.b);
}

// ADC A, C
void gbtest::LR35902::opcode89h()
{
    ADC_A(m_registers.c);
}

// ADC A, D
void gbtest::LR35902::opcode8Ah()
{
    ADC_A(m_registers.d);
}

// ADC A, E
void gbtest::LR35902::opcode8Bh()
{
    ADC_A(m_registers.e);
}

// ADC A, H
void gbtest::LR35902::opcode8Ch()
{
    ADC_A(m_registers.h);
}

// ADC A, L
void gbtest::LR35902::opcode8Dh()
{
    ADC_A(m_registers.l);
}

// ADC A, (HL)
void gbtest::LR35902::opcode8Eh()
{
    ADC_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// ADC A, A
void gbtest::LR35902::opcode8Fh()
{
    ADC_A(m_registers.a);
}

// SUB A, B
void gbtest::LR35902::opcode90h()
{
    SUB_A(m_registers.b);
}

// SUB A, C
void gbtest::LR35902::opcode91h()
{
    SUB_A(m_registers.c);
}

// SUB A, D
void gbtest::LR35902::opcode92h()
{
    SUB_A(m_registers.d);
}

// SUB A, E
void gbtest::LR35902::opcode93h()
{
    SUB_A(m_registers.e);
}

// SUB A, H
void gbtest::LR35902::opcode94h()
{
    SUB_A(m_registers.h);
}

// SUB A, L
void gbtest::LR35902::opcode95h()
{
    SUB_A(m_registers.l);
}

// SUB A, (HL)
void gbtest::LR35902::opcode96h()
{
    SUB_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// SUB A, A
void gbtest::LR35902::opcode97h()
{
    // No need to compute the values at runtime here
    m_registers.a = 0;

    m_registers.f.z = 1;
    m_registers.f.n = 1;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 4;
}

// SBC A, B
void gbtest::LR35902::opcode98h()
{
    SBC_A(m_registers.b);
}

// SBC A, C
void gbtest::LR35902::opcode99h()
{
    SBC_A(m_registers.c);
}

// SBC A, D
void gbtest::LR35902::opcode9Ah()
{
    SBC_A(m_registers.d);
}

// SBC A, E
void gbtest::LR35902::opcode9Bh()
{
    SBC_A(m_registers.e);
}

// SBC A, H
void gbtest::LR35902::opcode9Ch()
{
    SBC_A(m_registers.h);
}

// SBC A, L
void gbtest::LR35902::opcode9Dh()
{
    SBC_A(m_registers.l);
}

// SBC A, (HL)
void gbtest::LR35902::opcode9Eh()
{
    SBC_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// SBC A, A
void gbtest::LR35902::opcode9Fh()
{
    SBC_A(m_registers.a);
}

// AND A, B
void gbtest::LR35902::opcodeA0h()
{
    AND_A(m_registers.b);
}

// AND A, C
void gbtest::LR35902::opcodeA1h()
{
    AND_A(m_registers.c);
}

// AND A, D
void gbtest::LR35902::opcodeA2h()
{
    AND_A(m_registers.d);
}

// AND A, E
void gbtest::LR35902::opcodeA3h()
{
    AND_A(m_registers.e);
}

// AND A, H
void gbtest::LR35902::opcodeA4h()
{
    AND_A(m_registers.h);
}

// AND A, L
void gbtest::LR35902::opcodeA5h()
{
    AND_A(m_registers.l);
}

// AND A, (HL)
void gbtest::LR35902::opcodeA6h()
{
    AND_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// AND A, A
void gbtest::LR35902::opcodeA7h()
{
    AND_A(m_registers.a);
}

// XOR A, B
void gbtest::LR35902::opcodeA8h()
{
    XOR_A(m_registers.b);
}

// XOR A, C
void gbtest::LR35902::opcodeA9h()
{
    XOR_A(m_registers.c);
}

// XOR A, D
void gbtest::LR35902::opcodeAAh()
{
    XOR_A(m_registers.d);
}

// XOR A, E
void gbtest::LR35902::opcodeABh()
{
    XOR_A(m_registers.e);
}

// XOR A, H
void gbtest::LR35902::opcodeACh()
{
    XOR_A(m_registers.h);
}

// XOR A, L
void gbtest::LR35902::opcodeADh()
{
    XOR_A(m_registers.l);
}

// XOR A, (HL)
void gbtest::LR35902::opcodeAEh()
{
    XOR_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// XOR A, A
void gbtest::LR35902::opcodeAFh()
{
    XOR_A(m_registers.a);
}

// OR A, B
void gbtest::LR35902::opcodeB0h()
{
    OR_A(m_registers.b);
}

// OR A, C
void gbtest::LR35902::opcodeB1h()
{
    OR_A(m_registers.c);
}

// OR A, D
void gbtest::LR35902::opcodeB2h()
{
    OR_A(m_registers.d);
}

// OR A, E
void gbtest::LR35902::opcodeB3h()
{
    OR_A(m_registers.e);
}

// OR A, H
void gbtest::LR35902::opcodeB4h()
{
    OR_A(m_registers.h);
}

// OR A, L
void gbtest::LR35902::opcodeB5h()
{
    OR_A(m_registers.l);
}

// OR A, (HL)
void gbtest::LR35902::opcodeB6h()
{
    OR_A(m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU));
    m_cyclesToWait += 4;
}

// OR A, A
void gbtest::LR35902::opcodeB7h()
{
    OR_A(m_registers.a);
}

// CP A, B
void gbtest::LR35902::opcodeB8h()
{
    CP_A(m_registers.b);
}

// CP A, C
void gbtest::LR35902::opcodeB9h()
{
    CP_A(m_registers.c);
}

// CP A, D
void gbtest::LR35902::opcodeBAh()
{
    CP_A(m_registers.d);
}

// CP A, E
void gbtest::LR35902::opcodeBBh()
{
    CP_A(m_registers.e);
}

// CP A, H
void gbtest::LR35902::opcodeBCh()
{
    CP_A(m_registers.h);
}

// CP A, L
void gbtest::LR35902::opcodeBDh()
{
    CP_A(m_registers.l);
}

// CP A, (HL)
void gbtest::LR35902::opcodeBEh()
{
    const uint8_t val = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    CP_A(val);

    m_cyclesToWait += 4;
}

// CP A, A
void gbtest::LR35902::opcodeBFh()
{
    m_registers.f.z = 1;
    m_registers.f.n = 1;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 4;
}

// RET NZ
void gbtest::LR35902::opcodeC0h()
{
    if (m_registers.f.z) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 20;
}

// POP BC
void gbtest::LR35902::opcodeC1h()
{
    m_registers.bc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 12;
}

// JP NZ, a16
void gbtest::LR35902::opcodeC2h()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (m_registers.f.z) {
        m_cyclesToWait = 12;
        return;
    }

    m_registers.pc = val;
    m_cyclesToWait = 16;
}

// JP a16
void gbtest::LR35902::opcodeC3h()
{
    m_registers.pc = fetch() | (fetch() << 8);
    m_cyclesToWait = 16;
}

// CALL NZ, a16
void gbtest::LR35902::opcodeC4h()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (m_registers.f.z) {
        m_cyclesToWait = 12;
        return;
    }

    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = val;

    m_cyclesToWait = 24;
}

// PUSH BC
void gbtest::LR35902::opcodeC5h()
{
    m_bus.write(--m_registers.sp, m_registers.b, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.c, gbtest::BusRequestSource::CPU);

    m_cyclesToWait = 16;
}

// ADD A, d8
void gbtest::LR35902::opcodeC6h()
{
    ADD_A(fetch());
    m_cyclesToWait += 4;
}

// RST 00H
void gbtest::LR35902::opcodeC7h()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x00;

    m_cyclesToWait = 16;
}

// RET Z
void gbtest::LR35902::opcodeC8h()
{
    if (!m_registers.f.z) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 20;
}

// RET
void gbtest::LR35902::opcodeC9h()
{
    m_registers.pc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 16;
}

// JP Z, a16
void gbtest::LR35902::opcodeCAh()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (!m_registers.f.z) {
        m_cyclesToWait = 12;
        return;
    }

    m_registers.pc = val;
    m_cyclesToWait = 16;
}

// Prefixed instructions
void gbtest::LR35902::opcodeCBh()
{
    // Get the real opcode and the destination register/memory
    const uint8_t opcode = fetch();
    uint8_t lowOpcode = opcode & 0x7;

    auto getRegisterByLowerBits = [&](const uint8_t& lowerBits, uint8_t& defaultDest) -> uint8_t& {
        switch (lowerBits) {
        case 0x00:
            return m_registers.b;
        case 0x01:
            return m_registers.c;
        case 0x02:
            return m_registers.d;
        case 0x03:
            return m_registers.e;
        case 0x04:
            return m_registers.h;
        case 0x05:
            return m_registers.l;
        case 0x07:
            return m_registers.a;
        default:
            return defaultDest;
        }
    };

    uint8_t memValue = m_bus.read(m_registers.hl, gbtest::BusRequestSource::CPU);
    uint8_t& dest = getRegisterByLowerBits(lowOpcode, memValue);

    // Apply the right operation
    if (opcode >= 0x00 && opcode <= 0x07) {
        RLC(dest);
    }
    else if (opcode >= 0x08 && opcode <= 0x0F) {
        RRC(dest);
    }
    else if (opcode >= 0x10 && opcode <= 0x17) {
        RL(dest);
    }
    else if (opcode >= 0x18 && opcode <= 0x1F) {
        RR(dest);
    }
    else if (opcode >= 0x20 && opcode <= 0x27) {
        SLA(dest);
    }
    else if (opcode >= 0x28 && opcode <= 0x2F) {
        SRA(dest);
    }
    else if (opcode >= 0x30 && opcode <= 0x37) {
        SWAP(dest);
    }
    else if (opcode >= 0x38 && opcode <= 0x3F) {
        SRL(dest);
    }
    else if (opcode >= 0x40 && opcode <= 0x7F) {
        BIT((opcode - 0x40) / 0x8, dest);
    }
    else if (opcode >= 0x80 && opcode <= 0xBF) {
        RES((opcode - 0x80) / 0x8, dest);
    }
    else if (opcode >= 0xC0 && opcode <= 0xFF) {
        SET((opcode - 0xC0) / 0x8, dest);
    }

    // If the low opcode was equal to 6, then the destination was the memory
    if (lowOpcode == 0x06) {
        if (opcode < 0x40 || opcode > 0x7F) {
            // Only write the result if the operation was not BIT
            m_bus.write(m_registers.hl, memValue, gbtest::BusRequestSource::CPU);
        }

        m_cyclesToWait += 8;
    }
}

// CALL Z, a16
void gbtest::LR35902::opcodeCCh()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (!m_registers.f.z) {
        m_cyclesToWait = 12;
        return;
    }

    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = val;

    m_cyclesToWait = 24;
}

// CALL a16
void gbtest::LR35902::opcodeCDh()
{
    const uint16_t val = fetch() | (fetch() << 8);

    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = val;

    m_cyclesToWait = 24;
}

// ADC A, d8
void gbtest::LR35902::opcodeCEh()
{
    ADC_A(fetch());
    m_cyclesToWait += 4;
}

// RST 08H
void gbtest::LR35902::opcodeCFh()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x08;

    m_cyclesToWait = 16;
}

// RET NC
void gbtest::LR35902::opcodeD0h()
{
    if (m_registers.f.c) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 20;
}

// POP DE
void gbtest::LR35902::opcodeD1h()
{
    m_registers.de = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 12;
}

// JP NC, a16
void gbtest::LR35902::opcodeD2h()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (m_registers.f.c) {
        m_cyclesToWait = 12;
        return;
    }

    m_registers.pc = val;
    m_cyclesToWait = 16;
}

void gbtest::LR35902::opcodeD3h()
{
    throw std::runtime_error("Opcode not implemented!");
}

// CALL NC, a16
void gbtest::LR35902::opcodeD4h()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (m_registers.f.c) {
        m_cyclesToWait = 12;
        return;
    }

    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = val;

    m_cyclesToWait = 24;
}

// PUSH DE
void gbtest::LR35902::opcodeD5h()
{
    m_bus.write(--m_registers.sp, m_registers.d, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.e, gbtest::BusRequestSource::CPU);

    m_cyclesToWait = 16;
}

// SUB A, d8
void gbtest::LR35902::opcodeD6h()
{
    SUB_A(fetch());
    m_cyclesToWait += 4;
}

// RST 10H
void gbtest::LR35902::opcodeD7h()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x10;

    m_cyclesToWait = 16;
}

// RET C
void gbtest::LR35902::opcodeD8h()
{
    if (!m_registers.f.c) {
        m_cyclesToWait = 8;
        return;
    }

    m_registers.pc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 20;
}

// RETI
void gbtest::LR35902::opcodeD9h()
{
    m_interruptController.setInterruptMasterEnable(true);
    m_registers.pc = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);

    m_cyclesToWait = 16;
}

// JP C, a16
void gbtest::LR35902::opcodeDAh()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (!m_registers.f.c) {
        m_cyclesToWait = 12;
        return;
    }

    m_registers.pc = val;
    m_cyclesToWait = 16;
}

void gbtest::LR35902::opcodeDBh()
{
    throw std::runtime_error("Opcode not implemented!");
}

// CALL C, a16
void gbtest::LR35902::opcodeDCh()
{
    const uint16_t val = fetch() | (fetch() << 8);

    if (!m_registers.f.c) {
        m_cyclesToWait = 12;
        return;
    }

    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = val;

    m_cyclesToWait = 24;
}

void gbtest::LR35902::opcodeDDh()
{
    throw std::runtime_error("Opcode not implemented!");
}

// SBC A, d8
void gbtest::LR35902::opcodeDEh()
{
    SBC_A(fetch());
    m_cyclesToWait += 4;
}

// RST 18H
void gbtest::LR35902::opcodeDFh()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x18;

    m_cyclesToWait = 16;
}

// LDH (a8), A
void gbtest::LR35902::opcodeE0h()
{
    m_bus.write(0xFF00 | fetch(), m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 12;
}

// POP HL
void gbtest::LR35902::opcodeE1h()
{
    m_registers.hl = m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 12;
}

// LD (C), A
void gbtest::LR35902::opcodeE2h()
{
    m_bus.write(0xFF00 + m_registers.c, m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

void gbtest::LR35902::opcodeE3h()
{
    throw std::runtime_error("Opcode not implemented!");
}

void gbtest::LR35902::opcodeE4h()
{
    throw std::runtime_error("Opcode not implemented!");
}

// PUSH HL
void gbtest::LR35902::opcodeE5h()
{
    m_bus.write(--m_registers.sp, m_registers.h, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.l, gbtest::BusRequestSource::CPU);

    m_cyclesToWait = 16;
}

// AND A, d8
void gbtest::LR35902::opcodeE6h()
{
    m_registers.a &= fetch();

    m_registers.f.z = m_registers.a == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 1;
    m_registers.f.c = 0;

    m_cyclesToWait = 8;
}

// RST 20H
void gbtest::LR35902::opcodeE7h()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x20;

    m_cyclesToWait = 16;
}

// ADD SP, r8
void gbtest::LR35902::opcodeE8h()
{
    // First compute the final result
    const int8_t immediateValue = (int8_t) fetch();
    const uint32_t res = m_registers.sp + immediateValue;

    // Set the half-carry before doing anything as we need the current value in register A
    m_registers.f.h = ((((m_registers.sp & 0x000F) + (immediateValue & 0x0F)) & 0x0010) == 0x0010);
    m_registers.f.c = ((((m_registers.sp & 0x00FF) + (immediateValue & 0xFF)) & 0x0100) == 0x0100);

    // Set the accumulator to the result
    m_registers.sp = (res & 0xFFFF);

    // Set the flags according to the result
    m_registers.f.z = 0;
    m_registers.f.n = 0;

    m_cyclesToWait = 4;
}

// JP HL
void gbtest::LR35902::opcodeE9h()
{
    m_registers.pc = m_registers.hl;
    m_cyclesToWait = 4;
}

// LD (a16), A
void gbtest::LR35902::opcodeEAh()
{
    m_bus.write(fetch() | (fetch() << 8), m_registers.a, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 16;
}

void gbtest::LR35902::opcodeEBh()
{
    throw std::runtime_error("Opcode not implemented!");
}

void gbtest::LR35902::opcodeECh()
{
    throw std::runtime_error("Opcode not implemented!");
}

void gbtest::LR35902::opcodeEDh()
{
    throw std::runtime_error("Opcode not implemented!");
}

// XOR A, d8
void gbtest::LR35902::opcodeEEh()
{
    m_registers.a ^= fetch();

    m_registers.f.z = m_registers.a == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 8;
}

// RST 28H
void gbtest::LR35902::opcodeEFh()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x28;

    m_cyclesToWait = 16;
}

// LDH A, (a8)
void gbtest::LR35902::opcodeF0h()
{
    m_registers.a = m_bus.read(0xFF00 | fetch(), gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 12;
}

// POP AF
void gbtest::LR35902::opcodeF1h()
{
    m_registers.af = (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) & 0xF0)
            | (m_bus.read(m_registers.sp++, gbtest::BusRequestSource::CPU) << 8);
    m_cyclesToWait = 12;
}

// LD A, (C)
void gbtest::LR35902::opcodeF2h()
{
    m_registers.a = m_bus.read(0xFF00 + m_registers.c, gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 8;
}

// DI
void gbtest::LR35902::opcodeF3h()
{
    m_interruptController.setInterruptMasterEnable(false);
    m_cyclesToWait = 4;
}

void gbtest::LR35902::opcodeF4h()
{
    throw std::runtime_error("Opcode not implemented!");
}

// PUSH AF
void gbtest::LR35902::opcodeF5h()
{
    m_bus.write(--m_registers.sp, m_registers.af >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.af, gbtest::BusRequestSource::CPU);

    m_cyclesToWait = 16;
}

// OR A, d8
void gbtest::LR35902::opcodeF6h()
{
    m_registers.a |= fetch();

    m_registers.f.z = m_registers.a == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 8;
}

// RST 30H
void gbtest::LR35902::opcodeF7h()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x30;

    m_cyclesToWait = 16;
}

// LD HL, SP + r8
void gbtest::LR35902::opcodeF8h()
{
    auto a = (int8_t) fetch();
    m_registers.hl = m_registers.sp + a;

    m_registers.f.z = 0;
    m_registers.f.n = 0;
    m_registers.f.h = (((m_registers.sp & 0xF) + (a & 0xF)) & 0x10) == 0x10;
    m_registers.f.c = (((m_registers.sp & 0xFF) + (a & 0xFF)) & 0x100) == 0x100;

    m_cyclesToWait = 12;
}

// LD SP, HL
void gbtest::LR35902::opcodeF9h()
{
    m_registers.sp = m_registers.hl;
    m_cyclesToWait = 8;
}

// LD A, (a16)
void gbtest::LR35902::opcodeFAh()
{
    m_registers.a = m_bus.read(fetch() | (fetch() << 8), gbtest::BusRequestSource::CPU);
    m_cyclesToWait = 16;
}

// EI
void gbtest::LR35902::opcodeFBh()
{
    // Interrupt enable is delayed
    m_interruptController.setDelayedInterruptEnableCountdown(2);
    m_cyclesToWait = 4;
}

void gbtest::LR35902::opcodeFCh()
{
    throw std::runtime_error("Opcode not implemented!");
}

void gbtest::LR35902::opcodeFDh()
{
    throw std::runtime_error("Opcode not implemented!");
}

// CP A, d8
void gbtest::LR35902::opcodeFEh()
{
    const uint8_t val = fetch();
    CP_A(val);

    m_cyclesToWait += 4;
}

// RST 38H
void gbtest::LR35902::opcodeFFh()
{
    m_bus.write(--m_registers.sp, m_registers.pc >> 8, gbtest::BusRequestSource::CPU);
    m_bus.write(--m_registers.sp, m_registers.pc, gbtest::BusRequestSource::CPU);

    m_registers.pc = 0x38;

    m_cyclesToWait = 16;
}

// 0xCB-prefixed instructions
void gbtest::LR35902::RLC(uint8_t& dest)
{
    m_registers.f.c = (dest >> 7) & 0x1;

    dest = (dest << 1) | m_registers.f.c;

    m_registers.f.z = dest == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::RRC(uint8_t& dest)
{
    m_registers.f.c = dest & 0x1;

    dest = (dest >> 1) | (m_registers.f.c << 7);

    m_registers.f.z = dest == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::RL(uint8_t& dest)
{
    const uint8_t newCarry = (dest >> 7) & 0x1;

    dest = (dest << 1) | (m_registers.f.c & 0x1);

    m_registers.f.z = (dest == 0);
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = newCarry;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::RR(uint8_t& dest)
{
    const uint8_t newCarry = dest & 0x1;

    dest = (dest >> 1) | (m_registers.f.c << 7);

    m_registers.f.z = (dest == 0);
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = newCarry;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::SLA(uint8_t& dest)
{
    m_registers.f.c = (dest >> 7) & 0x1;

    dest <<= 1;

    m_registers.f.z = dest == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::SRA(uint8_t& dest)
{
    m_registers.f.c = dest & 0x1;

    dest >>= 1;
    dest |= (dest & 0x40) << 1;

    m_registers.f.z = dest == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::SWAP(uint8_t& dest)
{
    const uint8_t newUpper = dest << 4;
    dest = (dest >> 4) | newUpper;

    m_registers.f.z = dest == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::SRL(uint8_t& dest)
{
    m_registers.f.c = dest & 0x1;

    dest >>= 1;

    m_registers.f.z = dest == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 0;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::BIT(const uint8_t& bitToTest, const uint8_t& src)
{
    m_registers.f.z = (src & (1 << bitToTest)) == 0;
    m_registers.f.n = 0;
    m_registers.f.h = 1;

    m_cyclesToWait = 8;
}

void gbtest::LR35902::RES(const uint8_t& bitToClear, uint8_t& dest)
{
    dest &= ~(1 << bitToClear);
}

void gbtest::LR35902::SET(const uint8_t& bitToSet, uint8_t& dest)
{
    dest |= 1 << bitToSet;
}

void gbtest::LR35902::ADD_A(const uint8_t& src)
{
    // First compute the final result
    const uint16_t res = m_registers.a + src;

    // Set the half-carry before doing anything as we need the current value in register A
    m_registers.f.h = ((((m_registers.a & 0x0F) + (src & 0x0F)) & 0x10) == 0x10);

    // Set the accumulator to the result
    m_registers.a = res;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 0;
    m_registers.f.c = (res > 0xFF);

    m_cyclesToWait = 4;
}

void gbtest::LR35902::ADC_A(const uint8_t& src)
{
    // First compute the final result
    const uint16_t res = m_registers.a + src + m_registers.f.c;

    // We must compute the operation separately to set the half-carry properly
    m_registers.f.h = ((((m_registers.a & 0x0F) + (src & 0x0F) + (m_registers.f.c & 0x0F)) & 0x10) == 0x10);

    // Set the accumulator to the result
    m_registers.a = res;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 0;
    m_registers.f.c = (res > 0xFF);

    m_cyclesToWait = 4;
}

void gbtest::LR35902::SUB_A(const uint8_t& src)
{
    // Set (half-)carry flags before the operation takes place
    m_registers.f.h = (src & 0x0F) > (m_registers.a & 0x0F);
    m_registers.f.c = (src > m_registers.a);

    // Set the accumulator to the result
    m_registers.a -= src;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 1;

    m_cyclesToWait = 4;
}

void gbtest::LR35902::SBC_A(const uint8_t& src)
{
    // Set the (half-)carry before doing anything as we need the current value in register A
    const uint8_t oldCarry = m_registers.f.c;
    m_registers.f.h = (((src & 0x0F) + (oldCarry & 0x0F))) > (m_registers.a & 0x0F);
    m_registers.f.c = ((src + oldCarry) > m_registers.a);

    // Set the accumulator to the result
    m_registers.a -= src + oldCarry;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 1;

    m_cyclesToWait = 4;
}

void gbtest::LR35902::AND_A(const uint8_t& src)
{
    // Perform the operation
    m_registers.a &= src;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 0;
    m_registers.f.h = 1;
    m_registers.f.c = 0;

    m_cyclesToWait = 4;
}

void gbtest::LR35902::XOR_A(const uint8_t& src)
{
    // Perform the operation
    m_registers.a ^= src;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 4;
}

void gbtest::LR35902::OR_A(const uint8_t& src)
{
    // Perform the operation
    m_registers.a |= src;

    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == 0);
    m_registers.f.n = 0;
    m_registers.f.h = 0;
    m_registers.f.c = 0;

    m_cyclesToWait = 4;
}

void gbtest::LR35902::CP_A(const uint8_t& src)
{
    // Set the flags according to the result
    m_registers.f.z = (m_registers.a == src);
    m_registers.f.n = 1;
    m_registers.f.h = ((src & 0x0F) > (m_registers.a & 0x0F));
    m_registers.f.c = (src > m_registers.a);

    m_cyclesToWait = 4;
}

void gbtest::LR35902::INC_r8(uint8_t& reg)
{
    // Increment the register
    uint8_t oldVal = reg++;

    // Set the flags according to the result
    m_registers.f.z = (reg == 0);
    m_registers.f.n = 0;
    m_registers.f.h = ((oldVal & 0x08) && !(reg & 0x08));

    m_cyclesToWait = 4;
}

void gbtest::LR35902::DEC_r8(uint8_t& reg)
{
    // Decrement the register
    uint8_t oldVal = reg--;

    // Set the flags according to the result
    m_registers.f.z = (reg == 0);
    m_registers.f.n = 1;
    m_registers.f.h = ((oldVal & 0x10) != (reg & 0x10));

    m_cyclesToWait = 4;
}

void gbtest::LR35902::ADD_HL_r16(uint16_t& reg)
{
    // Add the value of the specified register to HL
    uint16_t oldVal = m_registers.hl;
    m_registers.hl += reg;

    // Set the flags according to the result
    m_registers.f.n = 0;
    m_registers.f.h = ((((oldVal & 0x0FFF) + (reg & 0x0FFF)) & 0x1000) == 0x1000);
    m_registers.f.c = ((((oldVal & 0xFFFF) + (reg & 0xFFFF)) & 0x10000) == 0x10000);

    m_cyclesToWait = 8;
}

void gbtest::LR35902::lookUp(uint8_t code) {
    switch (code) {
        case 0:
            opcode00h();
            break;
        case 1:
            opcode01h();
            break;
        case 2:
            opcode02h();
            break;
        case 3:
            opcode03h();
            break;
        case 4:
            opcode04h();
            break;
        case 5:
            opcode05h();
            break;
        case 6:
            opcode06h();
            break;
        case 7:
            opcode07h();
            break;
        case 8:
            opcode08h();
            break;
        case 9:
            opcode09h();
            break;
        case 10:
            opcode0Ah();
            break;
        case 11:
            opcode0Bh();
            break;
        case 12:
            opcode0Ch();
            break;
        case 13:
            opcode0Dh();
            break;
        case 14:
            opcode0Eh();
            break;
        case 15:
            opcode0Fh();
            break;
        case 16:
            opcode10h();
            break;
        case 17:
            opcode11h();
            break;
        case 18:
            opcode12h();
            break;
        case 19:
            opcode13h();
            break;
        case 20:
            opcode14h();
            break;
        case 21:
            opcode15h();
            break;
        case 22:
            opcode16h();
            break;
        case 23:
            opcode17h();
            break;
        case 24:
            opcode18h();
            break;
        case 25:
            opcode19h();
            break;
        case 26:
            opcode1Ah();
            break;
        case 27:
            opcode1Bh();
            break;
        case 28:
            opcode1Ch();
            break;
        case 29:
            opcode1Dh();
            break;
        case 30:
            opcode1Eh();
            break;
        case 31:
            opcode1Fh();
            break;
        case 32:
            opcode20h();
            break;
        case 33:
            opcode21h();
            break;
        case 34:
            opcode22h();
            break;
        case 35:
            opcode23h();
            break;
        case 36:
            opcode24h();
            break;
        case 37:
            opcode25h();
            break;
        case 38:
            opcode26h();
            break;
        case 39:
            opcode27h();
            break;
        case 40:
            opcode28h();
            break;
        case 41:
            opcode29h();
            break;
        case 42:
            opcode2Ah();
            break;
        case 43:
            opcode2Bh();
            break;
        case 44:
            opcode2Ch();
            break;
        case 45:
            opcode2Dh();
            break;
        case 46:
            opcode2Eh();
            break;
        case 47:
            opcode2Fh();
            break;
        case 48:
            opcode30h();
            break;
        case 49:
            opcode31h();
            break;
        case 50:
            opcode32h();
            break;
        case 51:
            opcode33h();
            break;
        case 52:
            opcode34h();
            break;
        case 53:
            opcode35h();
            break;
        case 54:
            opcode36h();
            break;
        case 55:
            opcode37h();
            break;
        case 56:
            opcode38h();
            break;
        case 57:
            opcode39h();
            break;
        case 58:
            opcode3Ah();
            break;
        case 59:
            opcode3Bh();
            break;
        case 60:
            opcode3Ch();
            break;
        case 61:
            opcode3Dh();
            break;
        case 62:
            opcode3Eh();
            break;
        case 63:
            opcode3Fh();
            break;
        case 64:
            opcode40h();
            break;
        case 65:
            opcode41h();
            break;
        case 66:
            opcode42h();
            break;
        case 67:
            opcode43h();
            break;
        case 68:
            opcode44h();
            break;
        case 69:
            opcode45h();
            break;
        case 70:
            opcode46h();
            break;
        case 71:
            opcode47h();
            break;
        case 72:
            opcode48h();
            break;
        case 73:
            opcode49h();
            break;
        case 74:
            opcode4Ah();
            break;
        case 75:
            opcode4Bh();
            break;
        case 76:
            opcode4Ch();
            break;
        case 77:
            opcode4Dh();
            break;
        case 78:
            opcode4Eh();
            break;
        case 79:
            opcode4Fh();
            break;
        case 80:
            opcode50h();
            break;
        case 81:
            opcode51h();
            break;
        case 82:
            opcode52h();
            break;
        case 83:
            opcode53h();
            break;
        case 84:
            opcode54h();
            break;
        case 85:
            opcode55h();
            break;
        case 86:
            opcode56h();
            break;
        case 87:
            opcode57h();
            break;
        case 88:
            opcode58h();
            break;
        case 89:
            opcode59h();
            break;
        case 90:
            opcode5Ah();
            break;
        case 91:
            opcode5Bh();
            break;
        case 92:
            opcode5Ch();
            break;
        case 93:
            opcode5Dh();
            break;
        case 94:
            opcode5Eh();
            break;
        case 95:
            opcode5Fh();
            break;
        case 96:
            opcode60h();
            break;
        case 97:
            opcode61h();
            break;
        case 98:
            opcode62h();
            break;
        case 99:
            opcode63h();
            break;
        case 100:
            opcode64h();
            break;
        case 101:
            opcode65h();
            break;
        case 102:
            opcode66h();
            break;
        case 103:
            opcode67h();
            break;
        case 104:
            opcode68h();
            break;
        case 105:
            opcode69h();
            break;
        case 106:
            opcode6Ah();
            break;
        case 107:
            opcode6Bh();
            break;
        case 108:
            opcode6Ch();
            break;
        case 109:
            opcode6Dh();
            break;
        case 110:
            opcode6Eh();
            break;
        case 111:
            opcode6Fh();
            break;
        case 112:
            opcode70h();
            break;
        case 113:
            opcode71h();
            break;
        case 114:
            opcode72h();
            break;
        case 115:
            opcode73h();
            break;
        case 116:
            opcode74h();
            break;
        case 117:
            opcode75h();
            break;
        case 118:
            opcode76h();
            break;
        case 119:
            opcode77h();
            break;
        case 120:
            opcode78h();
            break;
        case 121:
            opcode79h();
            break;
        case 122:
            opcode7Ah();
            break;
        case 123:
            opcode7Bh();
            break;
        case 124:
            opcode7Ch();
            break;
        case 125:
            opcode7Dh();
            break;
        case 126:
            opcode7Eh();
            break;
        case 127:
            opcode7Fh();
            break;
        case 128:
            opcode80h();
            break;
        case 129:
            opcode81h();
            break;
        case 130:
            opcode82h();
            break;
        case 131:
            opcode83h();
            break;
        case 132:
            opcode84h();
            break;
        case 133:
            opcode85h();
            break;
        case 134:
            opcode86h();
            break;
        case 135:
            opcode87h();
            break;
        case 136:
            opcode88h();
            break;
        case 137:
            opcode89h();
            break;
        case 138:
            opcode8Ah();
            break;
        case 139:
            opcode8Bh();
            break;
        case 140:
            opcode8Ch();
            break;
        case 141:
            opcode8Dh();
            break;
        case 142:
            opcode8Eh();
            break;
        case 143:
            opcode8Fh();
            break;
        case 144:
            opcode90h();
            break;
        case 145:
            opcode91h();
            break;
        case 146:
            opcode92h();
            break;
        case 147:
            opcode93h();
            break;
        case 148:
            opcode94h();
            break;
        case 149:
            opcode95h();
            break;
        case 150:
            opcode96h();
            break;
        case 151:
            opcode97h();
            break;
        case 152:
            opcode98h();
            break;
        case 153:
            opcode99h();
            break;
        case 154:
            opcode9Ah();
            break;
        case 155:
            opcode9Bh();
            break;
        case 156:
            opcode9Ch();
            break;
        case 157:
            opcode9Dh();
            break;
        case 158:
            opcode9Eh();
            break;
        case 159:
            opcode9Fh();
            break;
        case 160:
            opcodeA0h();
            break;
        case 161:
            opcodeA1h();
            break;
        case 162:
            opcodeA2h();
            break;
        case 163:
            opcodeA3h();
            break;
        case 164:
            opcodeA4h();
            break;
        case 165:
            opcodeA5h();
            break;
        case 166:
            opcodeA6h();
            break;
        case 167:
            opcodeA7h();
            break;
        case 168:
            opcodeA8h();
            break;
        case 169:
            opcodeA9h();
            break;
        case 170:
            opcodeAAh();
            break;
        case 171:
            opcodeABh();
            break;
        case 172:
            opcodeACh();
            break;
        case 173:
            opcodeADh();
            break;
        case 174:
            opcodeAEh();
            break;
        case 175:
            opcodeAFh();
            break;
        case 176:
            opcodeB0h();
            break;
        case 177:
            opcodeB1h();
            break;
        case 178:
            opcodeB2h();
            break;
        case 179:
            opcodeB3h();
            break;
        case 180:
            opcodeB4h();
            break;
        case 181:
            opcodeB5h();
            break;
        case 182:
            opcodeB6h();
            break;
        case 183:
            opcodeB7h();
            break;
        case 184:
            opcodeB8h();
            break;
        case 185:
            opcodeB9h();
            break;
        case 186:
            opcodeBAh();
            break;
        case 187:
            opcodeBBh();
            break;
        case 188:
            opcodeBCh();
            break;
        case 189:
            opcodeBDh();
            break;
        case 190:
            opcodeBEh();
            break;
        case 191:
            opcodeBFh();
            break;
        case 192:
            opcodeC0h();
            break;
        case 193:
            opcodeC1h();
            break;
        case 194:
            opcodeC2h();
            break;
        case 195:
            opcodeC3h();
            break;
        case 196:
            opcodeC4h();
            break;
        case 197:
            opcodeC5h();
            break;
        case 198:
            opcodeC6h();
            break;
        case 199:
            opcodeC7h();
            break;
        case 200:
            opcodeC8h();
            break;
        case 201:
            opcodeC9h();
            break;
        case 202:
            opcodeCAh();
            break;
        case 203:
            opcodeCBh();
            break;
        case 204:
            opcodeCCh();
            break;
        case 205:
            opcodeCDh();
            break;
        case 206:
            opcodeCEh();
            break;
        case 207:
            opcodeCFh();
            break;
        case 208:
            opcodeD0h();
            break;
        case 209:
            opcodeD1h();
            break;
        case 210:
            opcodeD2h();
            break;
        case 211:
            opcodeD3h();
            break;
        case 212:
            opcodeD4h();
            break;
        case 213:
            opcodeD5h();
            break;
        case 214:
            opcodeD6h();
            break;
        case 215:
            opcodeD7h();
            break;
        case 216:
            opcodeD8h();
            break;
        case 217:
            opcodeD9h();
            break;
        case 218:
            opcodeDAh();
            break;
        case 219:
            opcodeDBh();
            break;
        case 220:
            opcodeDCh();
            break;
        case 221:
            opcodeDDh();
            break;
        case 222:
            opcodeDEh();
            break;
        case 223:
            opcodeDFh();
            break;
        case 224:
            opcodeE0h();
            break;
        case 225:
            opcodeE1h();
            break;
        case 226:
            opcodeE2h();
            break;
        case 227:
            opcodeE3h();
            break;
        case 228:
            opcodeE4h();
            break;
        case 229:
            opcodeE5h();
            break;
        case 230:
            opcodeE6h();
            break;
        case 231:
            opcodeE7h();
            break;
        case 232:
            opcodeE8h();
            break;
        case 233:
            opcodeE9h();
            break;
        case 234:
            opcodeEAh();
            break;
        case 235:
            opcodeEBh();
            break;
        case 236:
            opcodeECh();
            break;
        case 237:
            opcodeEDh();
            break;
        case 238:
            opcodeEEh();
            break;
        case 239:
            opcodeEFh();
            break;
        case 240:
            opcodeF0h();
            break;
        case 241:
            opcodeF1h();
            break;
        case 242:
            opcodeF2h();
            break;
        case 243:
            opcodeF3h();
            break;
        case 244:
            opcodeF4h();
            break;
        case 245:
            opcodeF5h();
            break;
        case 246:
            opcodeF6h();
            break;
        case 247:
            opcodeF7h();
            break;
        case 248:
            opcodeF8h();
            break;
        case 249:
            opcodeF9h();
            break;
        case 250:
            opcodeFAh();
            break;
        case 251:
            opcodeFBh();
            break;
        case 252:
            opcodeFCh();
            break;
        case 253:
            opcodeFDh();
            break;
        case 254:
            opcodeFEh();
            break;
        case 255:
            opcodeFFh();
            break;
    }
}
