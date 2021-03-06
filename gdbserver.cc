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
using namespace std;

#include <boost/tokenizer.hpp>
using namespace boost;

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "gdbserver.hh"


class PayloadTokenizer {
public:
    PayloadTokenizer(std::string str, const char *sep_)
    {
        boost::char_separator<char> sep(sep_);
        boost::tokenizer< boost::char_separator<char> > t(str, sep);

        boost::tokenizer< boost::char_separator<char> >::iterator i = t.begin();
        boost::tokenizer< boost::char_separator<char> >::iterator e = t.end();
        for (; i != e; i++)
            tok.push_back(*i);
    }

    size_t size(void) const
    {
        tok.size();
    }

    std::string operator[](int idx) const
    {
        return tok[idx];
    }
private:
    vector<string> tok;
};

/*
 * Helper functions
 */
static char
int_to_hex(int value)
{
    assert(0 <= value && value < 16);
    return value < 10 ? value + '0' : value + 'a' - 10;
}

static std::string
bin_to_hex(const char *bin, size_t bin_size)
{
    std::string hex;
    for (int i = 0; i < bin_size; i++) {
        char c = bin[i];
        hex.push_back(int_to_hex(c >> 4 & 0xf));
        hex.push_back(int_to_hex(c >> 0 & 0xf));
    }
    return hex;
}

static unsigned long long
str_to_int(const std::string &str, int base = 16)
{
    /* TODO Handle different endianess */
    return strtoull(str.c_str(), NULL, base);
}


namespace gdb {

void
Context::put_reg(uint16_t value)
{
    reg_str += bin_to_hex((char *)&value, sizeof(value));
}

void
Context::put_reg(uint32_t value)
{
    reg_str += bin_to_hex((char *)&value, sizeof(value));
}

void
Context::put_reg(uint64_t value)
{
    reg_str += bin_to_hex((char *)&value, sizeof(value));
}

void
Context::put_mem(char value)
{
    mem_str += int_to_hex(value >> 4 & 0xf);
    mem_str += int_to_hex(value >> 0 & 0xf);
}

const std::string &
Context::rd_one_reg(int reg_no)
{
    reg_str.clear();
    rd_reg(reg_no);
    return reg_str;
}

const std::string &
Context::rd_all_regs(void)
{
    reg_str.clear();
    for (int i = 0; i < num_regs; i++)
        rd_reg(i);
    return reg_str;
}

const std::string &
Context::rd_mem_size(addr_type addr, size_type size)
{
    mem_str.clear();
    for (size_type i = 0; i < size; i++)
        rd_mem(addr + i);
    return mem_str;
}

bool
Context::wr_mem_size(addr_type addr, size_type size, const char *data)
{
    bool success = true;
    for (int i = 0; i < size; i++)
        success = success && wr_mem(addr + i, data[i]);
    return success;
}


/* TODO: This constructor does not properly cleanup after itslef in the case
 * of an error and is not exception safe. Fix this. It does however report
 * all errors.
 */
Server::Server(context_ptr context, const char *port)
: context(context), target_state(TARGET_STATE_HALTED)
{
    int rc, fd;
    struct addrinfo h, *r;
    struct sockaddr_in addr;
    socklen_t addr_len;

    std::memset(&h, 0, sizeof(struct addrinfo));
    h.ai_family = AF_UNSPEC;
    h.ai_socktype = SOCK_STREAM;
    h.ai_flags = AI_PASSIVE;

    rc = getaddrinfo(NULL, port, &h, &r);
    EXPECT(rc != -1, gai_strerror(rc));

    fd = socket(r->ai_family, r->ai_socktype, r->ai_protocol);
    EXPECT_ERRNO(fd != -1);

    int optval = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    rc = bind(fd, r->ai_addr, r->ai_addrlen);
    EXPECT_ERRNO(rc != -1);
    freeaddrinfo(r);

    rc = listen(fd, 1);
    EXPECT_ERRNO(rc != -1);

    addr_len = sizeof(struct sockaddr_in);
    socket_fd = accept(fd, (struct sockaddr *)&addr, &addr_len);
    EXPECT_ERRNO(socket_fd != -1);
    close(fd);

    std::cout << "GDB: Accepted connection from "
              << inet_ntoa(addr.sin_addr) << std::endl;
    
    send_ack();
    EXPECT(got_ack(), "Failed to establish connection with client");
}

Server::~Server(void)
{
    close(socket_fd);
}

void
Server::send_payload(const payload_type &payload, int tries) const
{
    std::string packet;

    /* Build packet */
    packet.push_back('$');
    do {
        payload_type::const_iterator i = payload.begin();
        payload_type::const_iterator e = payload.end();
        for (; i != e; i++) {
            char c = *i;

            /* Escape '$', '#' and '}' */
            switch (c) {
                case '$':
                case '#':
                case '}':
                    packet.push_back('}');
                    packet.push_back(c ^ 0x20);
                    break;
            }

            packet.push_back(c);
        }
    } while (0);
    packet.push_back('#');

    int c = compute_checksum(payload);
    packet.push_back(checksum_msb_ascii(c));
    packet.push_back(checksum_lsb_ascii(c));

    do {
        send_packet(packet.data(), packet.size());
        if (got_ack())
            break;
    } while (--tries);

    if (!tries)
        THROW("Failed to send packet.");
}

void
Server::send_packet(const char *buf, size_t buf_size) const
{
    const char *p = buf;
    size_t s = buf_size;

    while (s > 0) {
        ssize_t n = send(socket_fd, p, s, 0);
        EXPECT_ERRNO(n != -1);
        p += n;
        s -= n;
    }
}

void
Server::recv_payload(payload_type &payload, int tries) const
{
    packet_type packet;

    do {
        char p[PACKET_SIZE + 1];
        ssize_t size;

        size = recv_packet(p, PACKET_SIZE);
        EXPECT_ERRNO(size != -1);
        p[size] = '\0';

        assert(packet.empty());
        packet = p;

        if (extract_payload(packet, payload))
            break;
        send_nak();
    } while (--tries);

    if (!tries)
        THROW("Failed to receive valid packet.");
    send_ack();
}

size_t
Server::recv_packet(char *buf, size_t buf_size) const
{
    ssize_t size;
    size = recv(socket_fd, buf, buf_size, 0);
    EXPECT_ERRNO(size != -1);
    return size;
}

bool
Server::extract_payload(const packet_type &packet, payload_type &payload) const
{
    std::string::const_iterator i = packet.begin();
    std::string::const_iterator e = packet.end();

    /* Packet must begin with a '$' */
    if (i == e || *(i++) != '$')
        return false;

    for (; i != e && *i != '#'; i++){
        char c = *i;

        /* Handle escaped characters */
        if (c == '}') {
            c = *(++i) ^ 0x20;
        }
        payload.push_back(c);
    }

    /* Payload must be followed by a '#' */
    if (i == e || *(i++) != '#')
        return false;

    /* Extract and verify checksum */
    if (i == e)
        return false;
    char checksum_msb = *(i++);

    if (i == e)
        return false;
    char checksum_lsb = *(i++);

    /* TODO Verify checksum */
    return true;
}

int
Server::compute_checksum(const payload_type &payload) const
{
    int checksum = 0;
    payload_type::const_iterator i = payload.begin();
    payload_type::const_iterator e = payload.end();
    for (; i != e; i++)
        checksum += *i;
    return checksum % 256;
}

char
Server::checksum_lsb_ascii(int csum) const
{
    return int_to_hex(csum >> 0 & 0xf);
}

char
Server::checksum_msb_ascii(int csum) const
{
    return int_to_hex(csum >> 4 & 0xf);
}

void
Server::send_ack(void) const
{
    send_packet("+", 1);
}

void
Server::send_nak(void) const
{
    send_packet("-", 1);
}

bool
Server::got_ack(void) const
{
    char buf;
    recv_packet(&buf, 1);
    return buf == '+';
}

bool
Server::got_nak(void) const
{
    char buf;
    recv_packet(&buf, 1);
    return buf == '-';
}

void
Server::send_ok(void) const
{
    send_payload("OK");
}

void
Server::send_empty(void) const
{
    send_payload("");
}

void
Server::send_trapped(void) const
{
    send_payload("S05");
}

void
Server::send_error(int error) const
{
    stringstream ss;
    ss << "E" << error;
    send_payload(ss.str());
}

void
Server::wait_for_command(void)
{
    do {
        payload_type payload;
        recv_payload(payload);

        cout << "wait_for_comman: payload: " << payload << endl;

        switch(payload[0]) {
            case 'D':
                target_state = TARGET_STATE_DETACHED;
                send_ok();
                break;

            case 'g':
                send_payload(context->rd_all_regs());
                break;

            case 'H':
                if (payload.substr(1, 1) == "g") {
                    send_ok();

                } else if (payload.substr(1, 1) == "c") {
                    send_ok();

                } else {
                    THROW("Unsupported 'H' command");
                }
                break;

            case 'm':
                do {
                    PayloadTokenizer tok(payload.substr(1), ",:");
                    EXPECT(tok.size() == 2, "Packet format error. Command m.");

                    addr_type addr = str_to_int(tok[0]);
                    addr_type size = str_to_int(tok[1]);
                    send_payload(context->rd_mem_size(addr, size));
                } while (0);
                break;

            case 'M':
                do {
                    PayloadTokenizer tok(payload.substr(1), ",:");
                    EXPECT(tok.size() == 2, "Packet format error. Command M.");

                    addr_type addr = str_to_int(tok[0]);
                    addr_type size = str_to_int(tok[1]);
                    uint64_t data = str_to_int(tok[2]);
                    if (context->wr_mem_size(addr, size, (const char *)&data))
                        send_ok();
                    else
                        send_error(14);
                } while (0);
                break;

            case 'p':
                do {
                    PayloadTokenizer tok(payload.substr(1), "=");
                    EXPECT(tok.size() == 1, "Packet format error. Command p.");
                    send_payload(context->rd_one_reg(str_to_int(tok[0])));
                } while (0);
                break;

            case 'P':
                do {
                    PayloadTokenizer tok(payload.substr(1), "=");
                    EXPECT(tok.size() == 2, "Packet format error. Command P.");
                    context->wr_reg(str_to_int(tok[0]), str_to_int(tok[1]));
                    send_ok();
                } while (0);
                break;

            case 'q':
                if (payload.substr(1, 9) == "Supported") {
                    send_payload("PacketSize=1024;qXfer:features:read+");

                } else if (payload.substr(1, 7) == "Offsets") {
                    send_payload("Text=0;Data=0;Bss=0");

                } else if (payload.substr(1, 8) == "Attached") {
                    send_empty();

                } else if (payload.substr(1, 1) == "C") {
                    send_payload("QC1");

                } else if (payload.substr(1, 8) == "Symbol::") {
                    send_ok();

                } else if (payload.substr(1, 8) == "TStatus") {
                    send_empty();

                } else if (payload.substr(1, 29) == "Xfer:features:read:target.xml") {
                    send_payload("l" + context->xml_core());

                } else {
                    THROW("Unsupported 'q' command");
                }
                break;

            case 'v':
                if (payload.substr(1, 5) == "Cont?") {
                    send_payload("vCont;s;S;c;C");

                } else if (payload.substr(1, 6) == "Cont;c") {
                    target_state = TARGET_STATE_RUNNING;

                } else {
                    THROW("Unsupported 'v' command");
                }
                break;

            case 'z':
            case 'Z':
                if (payload.substr(1, 1) == "0") {
                    PayloadTokenizer tok(payload.substr(2), ",");
                    EXPECT(tok.size() == 2, "Packet format error (Z0)");

                    uint64_t addr = str_to_int(tok[0]);
                    uint64_t size = str_to_int(tok[1]);

                    if (payload[0] == 'z') 
                        context->del_breakpoint(addr, size);
                    else
                        context->set_breakpoint(addr, size);

                    send_ok();
                } else {
                    THROW("Unsupported 'z' command");
                }
                break;

            case 'X':
                /* We do not support binary data transfers */
                send_empty();
                break;

            case '?':
                /* We are always stopped by a SIGTRAP */
                send_trapped();
                break;

            default:
                THROW("Unsupported command");
                break;
        }
    } while (target_state == TARGET_STATE_HALTED);
}

void
Server::update(addr_type next_pc)
{
    switch (target_state) {
        case TARGET_STATE_HALTED:
            wait_for_command();
            break;

        case TARGET_STATE_RUNNING:
            if (context->has_breakpoint(next_pc)) {
                target_state = TARGET_STATE_HALTED;
                send_trapped(); /* Let the client know that we stopped */

                wait_for_command();
            }
            break;

        case TARGET_STATE_DETACHED:
            /* Do nothing */
            break;

        default:
            assert(0);
    }
}

} /* namespace gdb */
