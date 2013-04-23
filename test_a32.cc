/* Copyright (C) 2013, David Eklov
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cstring>

#include "gdbserver.hh"

using namespace std;
using namespace gdb;

#define TEXT_START              (0x8394)
#define TEXT_SIZE               (8 * 4)
#define TEXT_END                (TEXT_START + TEXT_SIZE)
#define TEXT_BRANCH_TARGET      (0x83a0)

class FakeARMv7Context : public Context {
public:
    FakeARMv7Context()
    {
        static char text_mem_[] = {
            0x04, 0xb0, 0x2d, 0xe5,  /* 8394:  push  {fp}            */
            0x00, 0xb0, 0x8d, 0xe2,  /* 8398:  add   fp, sp, #0      */
            0x14, 0xd0, 0x4d, 0xe2,  /* 839c:  sub   sp, sp, #20     */
            0x08, 0x20, 0x1b, 0xe5,  /* 83a0:  ldr   r2, [fp, #-8]   */
            0x0c, 0x30, 0x1b, 0xe5,  /* 83a4:  ldr   r3, [fp, #-12]  */
            0x03, 0x30, 0x82, 0xe0,  /* 83a8:  add   r3, r2, r3      */
            0x10, 0x30, 0x0b, 0xe5,  /* 83ac:  str   r3, [fp, #-16]  */
            0xfa, 0xff, 0xff, 0xea,  /* 83b0:  b     83a0            */
        };
        text_mem = text_mem_;

        regs[ARMv7_REG_PC] = TEXT_START;
    }

    void rd_reg(int reg_no)
    {
        put_reg(regs[reg_no]);
    }

    void rd_mem(uint64_t addr)
    {
        if (TEXT_START <= addr && addr <= TEXT_END) {
            addr -= TEXT_START;
            put_mem(text_mem[addr]);
        } else if (DATA_START <= addr && addr <= DATA_END) {
            addr -= DATA_START;
            put_mem(data_mem[addr]);
        } else 
            put_mem(0);
    }

    int num_regs(void)
    {
        return ARMv7_NUM_REGS;
    }
       
public:
    uint32_t regs[ARMv7_NUM_REGS];
    char *text_mem;
};

int
main(int argc, char **argv)
{
    FakeARMv7Context *ctx = new FakeARMv7Context();
    context_ptr ctx_ptr = context_ptr(ctx);

    Server server(ctx_ptr);

    try {
        Server::addr_type pc = TEXT_START;

        do {
            ctx->regs[ARMv7_REG_PC] = pc;
            server.update(pc);

            pc += 4;
            if (pc == TEXT_END)
                pc = TEXT_BRANCH_TARGET;
        } while (1);

    } catch (gdb::exception &e) {
        cerr << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    return 0;
}
