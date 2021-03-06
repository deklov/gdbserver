﻿/* Copyright (C) 2013, David Eklov
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

#ifndef __GDBSERVER_HH__
#define __GDBSERVER_HH__

#include <sstream>
#include <cstring> /* strerror */
#include <set>

#include <boost/shared_ptr.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdint.h>
#include <errno.h>

#define THROW(msg) do {                                          \
    std::stringstream s;                                         \
    s << "Error:" << __FILE__ << ":" << __LINE__ << ": " << msg; \
    throw exception(s.str());                                    \
} while(0)

#define EXPECT(c, m) do {               \
    if (!(c))                           \
        THROW(m);                       \
} while (0)

#define EXPECT_ERRNO(c) do {            \
    if (!(c))                           \
        THROW(std::strerror(errno));    \
} while (0)


namespace gdb {

    class exception : public std::exception {
    public:
        exception(std::string msg)
        : msg(msg) { }

        ~exception() throw()
        { }

        virtual const char *what() const throw()
        {
            return msg.c_str();
        }

    private:
        std::string msg;
    };

    enum ARMv7_RegisterNames {
        ARMv7_REG_R0 = 0,
        ARMv7_REG_R1,  ARMv7_REG_R2,  ARMv7_REG_R3,  ARMv7_REG_R4,
        ARMv7_REG_R5,  ARMv7_REG_R6,  ARMv7_REG_R7,  ARMv7_REG_R8,
        ARMv7_REG_R9,  ARMv7_REG_R10, ARMv7_REG_R11, ARMv7_REG_R12,
        ARMv7_REG_SP,  ARMv7_REG_LR,  ARMv7_REG_PC,  ARMv7_REG_F0,
        ARMv7_REG_F1,  ARMv7_REG_F2,  ARMv7_REG_F3,  ARMv7_REG_F4,
        ARMv7_REG_F5,  ARMv7_REG_F6,  ARMv7_REG_F7,  ARMv7_REG_FPS,
        ARMv7_REG_CPSR,

        ARMv7_NUM_REGS
    };

    /* TODO Move to .cc file and declare as extern */
    static const std::string armv7_xml_core =
        "<?xml version=\"1.0\"?>"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">"
        "<feature name=\"org.gnu.gdb.arm.core\">"
        "  <reg name=\"r0\"   bitsize=\"32\"/>"
        "  <reg name=\"r1\"   bitsize=\"32\"/>"
        "  <reg name=\"r2\"   bitsize=\"32\"/>"
        "  <reg name=\"r3\"   bitsize=\"32\"/>"
        "  <reg name=\"r4\"   bitsize=\"32\"/>"
        "  <reg name=\"r5\"   bitsize=\"32\"/>"
        "  <reg name=\"r6\"   bitsize=\"32\"/>"
        "  <reg name=\"r7\"   bitsize=\"32\"/>"
        "  <reg name=\"r8\"   bitsize=\"32\"/>"
        "  <reg name=\"r9\"   bitsize=\"32\"/>"
        "  <reg name=\"r10\"  bitsize=\"32\"/>"
        "  <reg name=\"r11\"  bitsize=\"32\"/>"
        "  <reg name=\"r12\"  bitsize=\"32\"/>"
        "  <reg name=\"sp\"   bitsize=\"32\" type=\"data_ptr\"/>"
        "  <reg name=\"lr\"   bitsize=\"32\"/>"
        "  <reg name=\"pc\"   bitsize=\"32\" type=\"code_ptr\"/>"
        "  <reg name=\"cpsr\" bitsize=\"32\" regnum=\"25\"/>"
        "</feature>";


    class Context {
        friend class Server;
    public:
        typedef uint64_t addr_type;
        typedef uint64_t size_type;

        Context(int num_regs)
        : num_regs(num_regs) { }

        /** */
        virtual void rd_reg(int reg_no) = 0;
        /** */
        virtual void wr_reg(int reg_no, unsigned long long value) = 0;
        /** */
        virtual void rd_mem(addr_type addr) = 0;
        /** */
        virtual bool wr_mem(addr_type addr, char data) = 0;
        /** */
        virtual void set_breakpoint(addr_type addr, size_type size = 1) = 0;
        /** */
        virtual void del_breakpoint(addr_type addr, size_type size = 1) = 0;
        /** */
        virtual bool has_breakpoint(addr_type addr, size_type size = 1) = 0;
        /** */
        virtual const std::string& xml_core(void) = 0;

    protected:
        void put_reg(uint16_t value);
        void put_reg(uint32_t value);
        void put_reg(uint64_t value);

        void put_mem(char value);

    private:
        const std::string& rd_one_reg(int reg_no);
        const std::string& rd_all_regs(void);

        const std::string& rd_mem_size(addr_type addr, size_type size);

        bool wr_mem_size(addr_type addr, size_type size, const char *data);

    private:
        std::string reg_str;
        std::string mem_str;

        int num_regs;
    };
    typedef boost::shared_ptr<Context> context_ptr;


    class Server {
    public:
        typedef uint64_t addr_type;
        typedef uint64_t addr_diff_type;

        Server(context_ptr context, const char *port = "1234");
        ~Server(void);

        void update(addr_type next_pc);

    private:
        typedef std::string payload_type;
        typedef std::string packet_type;

        enum {
            TARGET_STATE_HALTED = 0,
            TARGET_STATE_RUNNING,
            TARGET_STATE_DETACHED,
        } target_state;

        static const size_t PACKET_SIZE = 1024;

        void wait_for_command(void);

        bool extract_payload(const packet_type &packet, payload_type &payload) const;

        void send_payload(const payload_type &payload, int tries = 1) const;
        void send_packet(const char *buf, size_t buf_size) const;

        void recv_payload(payload_type &payload, int tries = 1) const;
        size_t recv_packet(char *buf, size_t buf_size) const;

        int compute_checksum(const payload_type &payload) const;
        char checksum_lsb_ascii(int checksum) const;
        char checksum_msb_ascii(int checksum) const;

        void send_ack(void) const;
        void send_nak(void) const;

        bool got_ack(void) const;
        bool got_nak(void) const;

        void send_ok(void) const;
        void send_empty(void) const;
        void send_error(int error) const;
        void send_trapped(void) const;

    private:
        context_ptr context;

        int socket_fd;
    };

} /* namespace gdb */

#endif /* __GDBSERVER_HH__ */
