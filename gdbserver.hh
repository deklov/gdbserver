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

    class Context {
    public:
        typedef uint64_t addr_type;
        typedef uint64_t size_type;

        /** */
        virtual void rd_reg(int reg_no) = 0;
        /** */
        virtual void rd_mem(uint64_t addr) = 0;
        /** */
        virtual int num_regs(void) = 0;

        const std::string& rd_one_reg(int reg_no);
        const std::string& rd_all_regs(void);

        const std::string& rd_mem(addr_type addr, size_type size);
                
    protected:
        void put_reg(uint16_t value);
        void put_reg(uint32_t value);
        void put_reg(uint64_t value);

        void put_mem(char value);

    private:
        std::string reg_str;
        std::string mem_str;
    };
    typedef boost::shared_ptr<Context> context_ptr;


    /** */
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
        } target_state;

        static const size_t PACKET_SIZE = 1024;

        void wait_for_command(void);

        void send_payload(const payload_type &payload, int tries = 1) const;
        void send_packet(const packet_type &packet) const;
        void send_packet(const char *buf, size_t buf_size) const;

        void recv_payload(payload_type &payload, int tries = 1) const;
        void recv_packet(payload_type &payload) const;
        size_t recv_packet(char *buf, size_t buf_size) const;

        void send_ack(void) const;
        void send_nak(void) const;

        bool got_ack(void) const;
        bool got_nak(void) const;

        void send_ok(void) const;
        void send_empty(void) const;
        void send_trapped(void) const;

        bool extract_payload(const packet_type &, payload_type &) const;

        int compute_checksum(const payload_type &payload) const;
        char checksum_lsb_ascii(int checksum) const;
        char checksum_msb_ascii(int checksum) const;

        void set_breakpoint(addr_type addr, addr_diff_type size = 1);
        void del_breakpoint(addr_type addr, addr_diff_type size = 1);
        bool has_breakpoint(addr_type addr, addr_diff_type size = 1);

        void handle_g(const payload_type &payload);
        void handle_H(const payload_type &payload);
        void handle_m(const payload_type &payload);
        void handle_p(const payload_type &payload);
        void handle_q(const payload_type &payload);
        void handle_v(const payload_type &payload);
        void handle_z(const payload_type &payload, bool set);
        void handle_qm(const payload_type &payload);

    private:
        context_ptr context;

        typedef std::set<addr_type> breakpoint_set_type;
        breakpoint_set_type breakpoint_set;

        int socket_fd;
    };

} /* namespace gdb */

#endif /* __GDBSERVER_HH__ */
