// nl_charls: reference encoder for nl_top. Splits a binary PGM into N vertical
// stripes and encodes each with CharLS at the given NEAR (default thresholds).
// Usage: nl_charls in.pgm near stripes out_prefix   -> out_prefix<s>.jls
#include <charls/charls.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static int next_int(std::istream& f)
{
    int c = f.get();
    for (;;) {
        while (isspace(c)) c = f.get();
        if (c != '#') break;
        while (c != '\n' && c != EOF) c = f.get();
    }
    int v = 0;
    while (isdigit(c)) { v = v * 10 + (c - '0'); c = f.get(); }
    return v; // consumes one whitespace after the number
}

int main(int argc, char** argv)
{
    if (argc != 5) { std::cerr << "usage: nl_charls in.pgm near stripes out_prefix\n"; return 2; }
    const int near = atoi(argv[2]), ns = atoi(argv[3]);
    std::ifstream f(argv[1], std::ios::binary);
    if (f.get() != 'P' || f.get() != '5') { std::cerr << "not a P5 PGM\n"; return 1; }
    const int w = next_int(f), h = next_int(f), mx = next_int(f);
    int bits = 0;
    while ((1 << bits) <= mx) ++bits;
    const int bps = mx > 255 ? 2 : 1;
    std::vector<uint8_t> raw(size_t(w) * h * bps);
    f.read(reinterpret_cast<char*>(raw.data()), raw.size());
    if (!f || w % ns) { std::cerr << "bad image or width not divisible\n"; return 1; }
    const int ws = w / ns;
    for (int s = 0; s < ns; ++s) {
        std::vector<uint8_t> buf(size_t(ws) * h * bps);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < ws; ++x) {
                const uint8_t* p = &raw[(size_t(y) * w + s * ws + x) * bps];
                uint8_t* q = &buf[(size_t(y) * ws + x) * bps];
                if (bps == 2) { q[0] = p[1]; q[1] = p[0]; } // PGM is big endian
                else q[0] = p[0];
            }
        charls::jpegls_encoder enc;
        enc.frame_info({uint32_t(ws), uint32_t(h), bits, 1}).near_lossless(near);
        std::vector<uint8_t> dst(enc.estimated_destination_size());
        enc.destination(dst);
        const size_t n = enc.encode(buf);
        std::ofstream o(std::string(argv[4]) + std::to_string(s) + ".jls", std::ios::binary);
        o.write(reinterpret_cast<const char*>(dst.data()), n);
    }
    return 0;
}
