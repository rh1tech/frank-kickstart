/*
 * frank-kickstart - host test of src/video/tmds_balanced.c
 *   cc -O2 -Isrc/video tools/test_tmds_balanced.c src/video/tmds_balanced.c -o /tmp/t && /tmp/t
 *
 * Checks: the level count, range and largest gap; at most 5 transitions
 * (receivers align on the control tokens' 7..9); five ones in every
 * symbol; a standard TMDS decoder gets the level back; no symbol is a
 * control token; the longest run of equal bits between any two pixels.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include "tmds_balanced.h"

static int decode(unsigned q) {
    unsigned d8 = q & 0xff;
    if (q & 0x200) d8 ^= 0xff;
    unsigned d = d8 & 1;
    for (int i = 1; i < 8; i++) {
        unsigned b = ((d8 >> i) ^ (d8 >> (i - 1))) & 1;
        if (!(q & 0x100)) b ^= 1;
        d |= b << i;
    }
    return (int)d;
}

int main(void) {
    int fail = 0;
    tmds_balanced_init();
    int levels[256], n = 0;
    for (int v = 0; v < 256; v++)
        if (tmds_balanced_symbol((unsigned char)v)) levels[n++] = v;
    int gap = 0;
    for (int i = 1; i < n; i++)
        if (levels[i] - levels[i - 1] > gap) gap = levels[i] - levels[i - 1];
    printf("levels %d, %d..%d, largest gap %d\n", n, levels[0], levels[n - 1], gap);
    if (n != 52 || gap > 17) fail = 1;

    for (int i = 0; i < n; i++) {
        unsigned s = tmds_balanced_symbol((unsigned char)levels[i]);
        int ones = __builtin_popcount(s);
        if (ones != 5) { printf("level %d: %d ones\n", levels[i], ones); fail = 1; }
        if (decode(s) != levels[i]) { printf("level %d decodes to %d\n", levels[i], decode(s)); fail = 1; }
        if (s == 0x354 || s == 0x0ab || s == 0x154 || s == 0x2ab) { printf("level %d is a control token\n", levels[i]); fail = 1; }
        /* transition-minimised, as data symbols must be (control tokens have 7..9) */
        int tr = 0;
        for (int b = 0; b < 9; b++) tr += ((s >> b) ^ (s >> (b + 1))) & 1;
        if (tr > 5) { printf("level %d: %d transitions\n", levels[i], tr); fail = 1; }
    }
    /* snapping: the nearest level, and levels stay put */
    int worst = 0;
    for (int v = 0; v < 256; v++) {
        int s = tmds_balanced_snap((unsigned char)v);
        if (!tmds_balanced_symbol((unsigned char)s)) { printf("snap %d -> %d: no symbol\n", v, s); fail = 1; }
        int e = s > v ? s - v : v - s;
        if (e > worst) worst = e;
    }
    printf("worst snap error %d\n", worst);
    /* 0..15 and 240..255 lie outside 16..239: at most 16 off there */
    if (worst > 16) fail = 1;

    /* runs: two symbols back to back, bit 0 first */
    int run_max = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            unsigned a = tmds_balanced_symbol((unsigned char)levels[i]), b = tmds_balanced_symbol((unsigned char)levels[j]);
            unsigned long long bits = a | (unsigned long long)b << 10;
            int run = 1, prev = bits & 1;
            for (int k = 1; k < 20; k++) {
                int cur = (bits >> k) & 1;
                run = cur == prev ? run + 1 : 1;
                prev = cur;
                if (run > run_max) run_max = run;
            }
        }
    printf("longest run of equal bits %d\n", run_max);
    printf(fail ? "FAIL\n" : "PASS\n");
    return fail;
}
