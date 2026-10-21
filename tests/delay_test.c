/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Copyright (C) 2026 Juan Ignacio Antonucci (Juan Jiant) */
/* (0.8.4) The delay bus's line (src/fx.c: dly_buf, dly_lo) on the host, through hostsim.c as regress.c.
 *   build/host/delay_test [DEMODIR]          (run_tests.sh: build/fx_demo)
 * 1. no leak at the far end: at the longest TIME with MOD up (1/1 at 120 BPM; the full rate's own end; the longest of
 *    all, plain, PING-PONG and WIDE), nothing comes out before the echo is due (until 0.8.4 the read wrapped onto the
 *    newest samples: the input leaked into the echo, -44 dB at MOD 20, -22 dB at MOD 42).
 * 2. the times: every long division at tempos 40 .. 200 BPM gets its own length up to the longest (printed against
 *    0.8.3's cap), the rate each runs at; the free times past 1.48 s.
 * 3. timing and level at each rate: the first echo TIME (+ the filters' few ms) after the send, the repeats exactly TIME
 *    apart, the echo's level the full rate's.
 * 4. the sound of each rate: a sine sweep through the echo (DIGITAL, FEEDBK 0): the gain, the largest alias / image;
 *    the residue of a 1 kHz sine at -12 and -40 dBFS (the noise floor), on the first repeat and the eighth (FEEDBK 100).
 * 5. no click when the rate changes: TIME swept across the boundaries while a sine plays (each way), the tempo changed,
 *    a jump from 1/8 to 4BAR (a section / preset load); an external clock wavering at a boundary does not restart the
 *    line each beat; after a change nothing of the old line is heard.
 * 6. at half and a quarter rate: MOD, the GRAIN (PITCH +12: an octave up), DIGITAL's held repeat (FEEDBK 100: no gain
 *    error through the filters), ANALOG's FEEDBK 120 bounded, PING-PONG's sides, TONE; the gate open for a long echo.
 * Demos (WAV) into DEMODIR: a pluck through 1/2 at 60 BPM (half rate) and 2BAR at 120 (a quarter). */
#define main hostsim_main
#include "hostsim.c"
#undef main

static int bad;
static void check(const char *what, int ok)
{
    printf("delay: %-94s %s\n", what, ok ? "ok" : "FAIL");
    bad += !ok;
}
#define NMAX (32u * FS)
static int32_t L[NMAX], R[NMAX];
static uint32_t xs = 0x2468aceu;
static int32_t noise(int32_t amp)
{
    xs ^= xs << 13;
    xs ^= xs >> 17;
    xs ^= xs << 5;
    return (int32_t)(((int64_t)(int32_t)xs * amp) >> 31);
}

static uint32_t g_now;                              /* (the next block's first sample: drun goes on from it, whole blocks) */
static void dreset(void)
{
    host_tracks_init();
    memset(dly_buf, 0, sizeof dly_buf);
    memset(cho_buf, 0, sizeof cho_buf);
    rev_clear();
    fx.dly_lp = fx.dly_hp = fx.dt1 = fx.dt2 = 0;
    fx.dl_xf = fx.dr_xf = 0;
    fx.gr_on = fx.gr_old = 0;
    fx.primed = 0;
    fx.rtype = 0;
    fxg_g = 32767; fxg_quiet = 0;
    song.g[G_DMIX] = 127; song.g[G_DFDBK] = 0; song.g[G_DCOLOR] = 64; song.g[G_DMOD] = 0; song.g[G_WIDTH] = 0;
    song.g[G_DPIT] = 0; song.g[G_DSPRY] = 0; song.g[G_DPING] = 0; song.g[G_DTYPE] = 0; song.g[G_RTYPE] = 0;
    song.g[G_CLOCK] = 0; midi_beat_samples = 0;
    song.g[G_BPM] = 120;
    g_now = 0;
}
static void at_samples(uint32_t q)                 /* TIME = q samples exactly: an external clock's quarter, 1/4 */
{
    song.g[G_CLOCK] = 1;
    midi_beat_samples = q;
    song.g[G_DTIME] = 0;
}
#ifdef DLY_SPAN
static uint32_t stage(void) { return fx.dr_sh; }
#endif
typedef int32_t (*src_t)(uint32_t t);
static void (*blk_hook)(uint32_t t);
/* len samples of the bus, the delay's send from src (the chorus and reverb silent): out L / R from t0 */
static void drun(src_t src, uint32_t t0, uint32_t len)
{
    static int32_t c[CTL], d[CTL], r[CTL], w[CTL];
    uint32_t t, i, end = t0 + len;
    for (t = g_now; t < end; t += CTL) {
        if (blk_hook) blk_hook(t);
        for (i = 0; i < CTL; i++) {
            c[i] = r[i] = 0;
            d[i] = src(t + i);
            fx_sl[i] = fx_sr[i] = 0;
        }
        fx_buses(c, d, r, w, CTL);
        for (i = 0; i < CTL; i++)
            if (t + i < NMAX) {
                L[t + i] = w[i] + fx_sl[i];
                R[t + i] = w[i] + fx_sr[i];
            }
    }
    g_now = t;
}
static double rms(const int32_t *x, uint32_t a, uint32_t b)
{
    double e = 0;
    uint32_t i;
    for (i = a; i < b; i++) e += (double)x[i] * x[i];
    return b > a ? sqrt(e / (b - a)) : 0;
}
static int32_t peak(const int32_t *x, uint32_t a, uint32_t b)
{
    int32_t p = 0;
    uint32_t i;
    for (i = a; i < b; i++) if (abs(x[i]) > p) p = abs(x[i]);
    return p;
}
static uint32_t argpeak(const int32_t *x, uint32_t a, uint32_t b)
{
    int32_t p = -1;
    uint32_t i, k = a;
    for (i = a; i < b; i++) if (abs(x[i]) > p) { p = abs(x[i]); k = i; }
    return k;
}

static double s_f, s_a;
static uint32_t s_len;
static int32_t sine_src(uint32_t t) { return t < s_len ? (int32_t)(s_a * sin(2.0 * M_PI * s_f * t / FS)) : 0; }
static int32_t sine_on(uint32_t t) { return (int32_t)(s_a * sin(2.0 * M_PI * s_f * t / FS)); }
static int32_t click_src(uint32_t t) { return t < 64u ? (int32_t)(30000.0 * sin(M_PI * t / 64.0)) : 0; }
static int32_t burst_src(uint32_t t) { return t < FS / 8u ? noise(30000) : 0; }

/* ------------------------------------------------------------------ 1. the leak --- */
/* the output while a 1 kHz sine is sent for 1 s, the echo not due yet (TIME >= 1.4 s), against the send (the echo comes
 * back at the send's level: MIX 127), the tape's WOW from its peak (the read furthest back) */
static double leak_db(int32_t mod, int ping, int width, double *at)
{
    uint32_t dl, end = FS + FS / 4u;
    double lk;
    song.g[G_DMOD] = (int16_t)mod; song.g[G_DPING] = (int16_t)ping; song.g[G_WIDTH] = (int16_t)width;
    fx.primed = 0;
    fx.dw_ph = 0x40000000u; fx.df_ph = 0x40000000u; fx.dw_m = 0;
    memset(dly_buf, 0, sizeof dly_buf);
    s_f = 1000; s_a = 40000; s_len = end;
    dl = delay_samples();
    *at = dl / (double)FS;
    {   /* (up to the echo's time: the echo due at TIME, PING-PONG's right at twice, WIDE's later) */
        uint32_t e = dl - 64u < FS ? dl - 64u : FS;
        drun(sine_src, 0, e);
        lk = rms(L, FS / 8u, e) > rms(R, FS / 8u, e) ? rms(L, FS / 8u, e) : rms(R, FS / 8u, e);
    }
    if (lk == 0)
        return -999;
    return 20 * log10(lk / (40000 / sqrt(2.0)));
}
static void test_leak(void)
{
    static const struct { const char *nm; int bpm, dt, mod, ping, wd; uint32_t q; } S[] = {
        {"1/1 at 120 BPM, MOD 20", 120, 7, 20, 0, 0, 0},
        {"1/1 at 120 BPM, MOD 42", 120, 7, 42, 0, 0, 0},
        {"1/1 at 120 BPM, MOD 127", 120, 7, 127, 0, 0, 0},
        {"4BAR at 40 BPM (the longest), MOD 127", 40, 9, 127, 0, 0, 0},
        {"4BAR at 40 BPM, MOD 127, PING-PONG", 40, 9, 127, 1, 0, 0},
        {"4BAR at 40 BPM, MOD 127, WIDE 127", 40, 9, 127, 0, 127, 0},
        {"65535 samples (0.8.3's cap), MOD 127", 120, 0, 127, 0, 0, 65535u},
#ifdef DLY_SPAN
        {"the full rate's own end (DLY_SPAN(0)), MOD 127", 120, 0, 127, 0, 0, DLY_SPAN(0u)},
        {"the half rate's own end (DLY_SPAN(1)), MOD 127", 120, 0, 127, 0, 0, DLY_SPAN(1u)},
#endif
    };
    uint32_t k;
    for (k = 0; k < NELEM(S); k++) {
        double at, db;
        char what[200];
        dreset();
        song.g[G_BPM] = (int16_t)S[k].bpm;
        song.g[G_DTIME] = (int16_t)S[k].dt;
        if (S[k].q) at_samples(S[k].q);
        db = leak_db(S[k].mod, S[k].ping, S[k].wd, &at);
        if (db <= -999)
            snprintf(what, sizeof what, "no leak at the far end: %s (TIME %.3f s): nothing before the echo", S[k].nm, at);
        else
            snprintf(what, sizeof what, "no leak at the far end: %s (TIME %.3f s): the input leaks in at %.1f dB", S[k].nm, at, db);
        check(what, db <= -999);
    }
}

#ifdef DLY_SPAN
/* ------------------------------------------------------------------ 2. the times --- */
static void test_times(void)
{
    static const int BPM[] = {40, 60, 80, 81, 100, 120, 140, 161, 162, 180, 200};
    static const uint8_t DV[4] = {6, 7, 8, 9};
    static const char *const RN[3] = {"full", "half", "1/4"};
    uint32_t b, k, ok = 1;
    printf("delay:   the long divisions: 0.8.3 (capped at 1.486 s) -> now [the line's rate]\n");
    printf("delay:   BPM   1/2                    1/1                    2BAR                   4BAR\n");
    for (b = 0; b < NELEM(BPM); b++) {
        printf("delay:   %3d ", BPM[b]);
        for (k = 0; k < 4u; k++) {
            uint32_t len, before, now, s;
            int32_t c[CTL] = {0}, w[CTL];
            dreset();
            song.g[G_BPM] = (int16_t)BPM[b];
            song.g[G_DTIME] = DV[k];
            len = div_samples(DV[k]);
            before = len >= 65536u ? 65535u : len;
            now = delay_samples();
            fx_buses(c, c, c, w, CTL);
            s = stage();
            ok &= now == (len > DLY_MAX ? DLY_MAX : len) && (s == 0u ? now <= DLY_SPAN(0u) : now > DLY_SPAN(s - 1u)) &&
                  now <= DLY_SPAN(s);
            printf("  %5.2f%s -> %5.2f%s %-4s ", before / (double)FS, len > before ? "*" : " ", now / (double)FS,
                   len > now ? "*" : " ", RN[s]);
        }
        printf("\n");
    }
    printf("delay:   (* capped)\n");
    check("every long division its own length up to the longest the line holds, at the lowest rate that holds it", ok);
    {
        uint32_t n = NELEM(DTIME_MS), last = DTIME_MS[n - 1u] * 441u / 10u, i, inc = 1;
        for (i = 1; i < n; i++) inc &= DTIME_MS[i] > DTIME_MS[i - 1u];
        dreset();
        song.g[G_DTIME] = (int16_t)(DT_FREE + n - 1u);
        printf("delay:   free times: %u, the last %s = %.3f s (the line's longest %.3f s); full rate to %.3f s, half to %.3f s\n",
               n, N_DTIME[DT_FREE + n - 1u], last / (double)FS, DLY_MAX / (double)FS, DLY_SPAN(0u) / (double)FS,
               DLY_SPAN(1u) / (double)FS);
        check("the free times past 1.48 s, each longer, the last within the line (its own length)",
              inc && n > 48u && DTIME_MS[47] == 1480u && last <= DLY_MAX && delay_samples() == last &&
              NELEM(N_DTIME) == DT_FREE + n && (uint32_t)GP[G_DTIME].max == DT_FREE + n - 1u);
    }
}

/* the TIME of each rate's test: within it, clear of its ends; t_lo: just past its start (the 8th repeats) */
static uint32_t t_of(uint32_t s) { return s == 0u ? DLY_SPAN(0u) * 3u / 5u : s == 1u ? DLY_SPAN(0u) * 3u / 2u : DLY_SPAN(1u) * 3u / 2u; }
static uint32_t t_lo(uint32_t s) { return s == 0u ? DLY_SPAN(0u) * 9u / 10u : DLY_SPAN(s - 1u) * 21u / 20u; }

/* ------------------------------------------------------- 3. timing and level --- */
static void gate_open(uint32_t t) { (void)t; fxg_quiet = 0; }
static void test_timing(void)
{
    uint32_t s;
    int32_t p0 = 0;
    for (s = 0; s < 3u; s++) {
        uint32_t dl = t_of(s), a, b2;
        int32_t p1, p2;
        char what[200];
        dreset();
        at_samples(dl);
        song.g[G_DTYPE] = 1; song.g[G_DFDBK] = 80;
        blk_hook = gate_open;                      /* (the FX gate out of it: test_features has it) */
        drun(click_src, 0, 2u * dl + FS / 4u);
        blk_hook = 0;
        a = argpeak(L, dl - 400u, dl + 400u);
        b2 = argpeak(L, 2u * dl - 400u, 2u * dl + 400u);
        p1 = peak(L, dl - 400u, dl + 400u);
        p2 = peak(L, 2u * dl - 400u, 2u * dl + 400u);
        if (!s) p0 = p1;
        snprintf(what, sizeof what, "rate %s, TIME %.3f s: the echo %+.2f ms late, the next %d samples on (TIME %u), level %+.2f dB",
                 s == 0u ? "full" : s == 1u ? "half" : "1/4", dl / (double)FS, ((double)a - 32 - dl) * 1000.0 / FS,
                 (int)(b2 - a), dl, 20 * log10((double)p1 / p0));
        check(what, stage() == s && a >= dl + 31u && a <= dl + 32u + (s ? 200u : 1u) && abs((int)(b2 - a) - (int)dl) <= 1 &&
                    fabs(20 * log10((double)p1 / p0)) < (s == 2u ? 1.5 : 0.5) && p2 > 0);
    }
}

/* ------------------------------------------------------------- 4. the sound --- */
#define FFTN 16384u
static double fre[FFTN], fim[FFTN];
static void fft(void)
{
    uint32_t i, j = 0, len, k;
    for (i = 1; i < FFTN; i++) {
        uint32_t bit = FFTN >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { double t = fre[i]; fre[i] = fre[j]; fre[j] = t; t = fim[i]; fim[i] = fim[j]; fim[j] = t; }
    }
    for (len = 2; len <= FFTN; len <<= 1) {
        double a = -2 * M_PI / len, wr = cos(a), wi = sin(a);
        for (i = 0; i < FFTN; i += len) {
            double cr = 1, ci = 0;
            for (k = 0; k < len / 2u; k++) {
                double ur = fre[i + k], ui = fim[i + k];
                double vr = fre[i + k + len / 2u] * cr - fim[i + k + len / 2u] * ci;
                double vi = fre[i + k + len / 2u] * ci + fim[i + k + len / 2u] * cr;
                double t;
                fre[i + k] = ur + vr; fim[i + k] = ui + vi;
                fre[i + k + len / 2u] = ur - vr; fim[i + k + len / 2u] = ui - vi;
                t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
            }
        }
    }
}
/* the spectrum of x[a .. a + FFTN) (Blackman-Harris: side lobes -92 dB), amplitudes in x's units: at f, the largest
 * other line (away from f and from DC), and everything else's RMS */
static void spectrum(const int32_t *x, uint32_t a, double f, double *at_f, double *spur, double *spur_hz, double *resid)
{
    uint32_t i, kf = (uint32_t)(f * FFTN / FS + 0.5);
    double cg = 0, e = 0, best = 0, bk = 0;
    for (i = 0; i < FFTN; i++) {
        double t = 2 * M_PI * i / FFTN, w = 0.35875 - 0.48829 * cos(t) + 0.14128 * cos(2 * t) - 0.01168 * cos(3 * t);
        fre[i] = x[a + i] * w; fim[i] = 0; cg += w;
    }
    fft();
    *at_f = 0;
    for (i = 1; i < FFTN / 2u; i++) {
        double m = 2 * sqrt(fre[i] * fre[i] + fim[i] * fim[i]) / cg;
        if (i + 6u >= kf && i <= kf + 6u) { if (m > *at_f) *at_f = m; continue; }
        if (i < 8u) continue;
        e += m * m / 2;
        if (m > best) { best = m; bk = i; }
    }
    *spur = best; *spur_hz = bk * FS / (double)FFTN;
    *resid = sqrt(e / 1.97);                       /* (Blackman-Harris's noise bandwidth, 2 bins) */
}
static void tone_through(uint32_t s, double f, double amp, uint32_t reps, double *g, double *sp, double *sph, double *res)
{
    uint32_t dl = reps > 1u ? t_lo(s) : t_of(s), a;
    double af;
    f = floor(f * FFTN / FS + 0.5) * FS / FFTN;    /* (on a bin: no scalloping) */
    dreset();
    at_samples(dl);
    song.g[G_DTYPE] = 1; song.g[G_DFDBK] = reps > 1u ? 100 : 0;
    if (reps > 1u) blk_hook = gate_open;
    s_f = f; s_a = amp * 2; s_len = FFTN + FS / 8u + 256u;   /* (shorter than TIME: the repeats apart) */
    a = dl * reps + FS / 8u;
    drun(sine_src, 0, a + FFTN);
    blk_hook = 0;
    spectrum(L, a, f, &af, sp, sph, res);
    *g = 20 * log10(af / amp + 1e-12);
    *sp = 20 * log10(*sp / amp + 1e-12);
    *res = 20 * log10(*res / 32767 + 1e-12);
}
static void test_sound(void)
{
    static const double F[] = {100, 1000, 3000, 4000, 4500, 5000, 6000, 8000, 9000, 9500, 10000, 10500, 11000, 12000,
                               13000, 15000, 18000, 20000};
    static const char *const RN[3] = {"full", "half", "1/4"};
    static const double PASS[3] = {20000, 9000, 4500};
    uint32_t s, k;
    double inband[3] = {-200, -200, -200}, img[3] = {-200, -200, -200}, g1k[3] = {0}, p01[3] = {0}, p1[3] = {0}, p3[3] = {0};
    printf("delay:   the echo of a sine at -12 dBFS (DIGITAL, FEEDBK 0): gain dB (1 kHz 0) / the largest other line, dB of the sine sent (Hz)\n");
    printf("delay:   Hz        full                     half                     1/4\n");
    for (s = 0; s < 3u; s++) {
        double g, sp, sph, res;
        tone_through(s, 1000, 8000, 1, &g1k[s], &sp, &sph, &res);
    }
    for (k = 0; k < NELEM(F); k++) {
        printf("delay:   %5.0f ", F[k]);
        for (s = 0; s < 3u; s++) {
            double g, sp, sph, res;
            tone_through(s, F[k], 8000, 1, &g, &sp, &sph, &res);
            g -= g1k[s];
            sp -= g1k[s];
            printf("   %+7.2f / %6.1f (%5.0f)", g, sp, sph);
            if (sph < PASS[s] && sp > inband[s]) inband[s] = sp;          /* (anything landing in the pass band) */
            if (F[k] <= PASS[s] && sp - g > img[s]) img[s] = sp - g;     /* (a pass band tone's images, of it) */
        }
        printf("\n");
    }
    for (s = 1; s < 3u; s++) {
        double f;
        for (f = s == 1u ? 8000 : 3000; f <= (s == 1u ? 11500 : 6000); f += 50) {
            double g, sp, sph, res;
            tone_through(s, f, 8000, 1, &g, &sp, &sph, &res);
            g -= g1k[s];
            if (g > -0.1) p01[s] = f;
            if (g > -1) p1[s] = f;
            if (g > -3) p3[s] = f;
        }
        printf("delay:   %s rate: flat (-0.1 dB) to %.0f Hz, -1 dB %.0f Hz, -3 dB %.0f Hz; any alias / image landing below %.0f Hz "
               "%.1f dB, a tone below it: its images %.1f dB of it\n", RN[s], p01[s], p1[s], p3[s], PASS[s], inband[s], img[s]);
    }
    {
        double r[3][4];
        uint32_t c;
        for (s = 0; s < 3u; s++) {
            double g, sp, sph;
            tone_through(s, 1000, 8000, 1, &g, &sp, &sph, &r[s][0]);
            tone_through(s, 1000, 80, 1, &g, &sp, &sph, &r[s][1]);
            tone_through(s, 1000, 8000, 8, &g, &sp, &sph, &r[s][2]);
            tone_through(s, 1000, 80, 8, &g, &sp, &sph, &r[s][3]);
        }
        printf("delay:   the residue (all but the tone) of a 1 kHz sine, dBFS: first repeat at -12 / -52 dBFS; 8th repeat (FEEDBK 100)"
               " at -12 / -52\n");
        for (s = 0; s < 3u; s++)
            printf("delay:     %-4s  %6.1f  %6.1f   %6.1f  %6.1f\n", RN[s], r[s][0], r[s][1], r[s][2], r[s][3]);
        c = 1;
        for (s = 1; s < 3u; s++)
            c &= r[s][0] < r[0][0] + 8 && r[s][1] < r[0][1] + 6 && r[s][2] < r[0][2] + 6 && r[s][0] < -80;
        check("half / quarter rate: the residue of a sine (noise, images) under -80 dBFS, within 8 dB of the full rate's", c);
    }
    check("half rate: flat to 9 kHz, -1 dB past 9.9 kHz; every alias / image landing in the pass band under -60 dB",
          p01[1] >= 9000 && p1[1] >= 9900 && inband[1] < -60 && img[1] < -60);
    check("quarter rate: flat to 4.5 kHz, -1 dB past 4.9 kHz; every alias / image landing in the pass band under -60 dB",
          p01[2] >= 4500 && p1[2] >= 4900 && inband[2] < -60 && img[2] < -60);
}

/* --------------------------------------------------------------- 5. the clicks --- */
/* the largest step sample to sample of the out during [a, b), against the steady signal's own */
static uint32_t step_at;
static int32_t step_max(uint32_t a, uint32_t b)
{
    int32_t m = 0;
    uint32_t i;
    for (i = a + 1u; i < b; i++) {
        int32_t d1 = abs(L[i] - L[i - 1u]), d2 = abs(R[i] - R[i - 1u]);
        if (d1 > m) { m = d1; step_at = i; }
        if (d2 > m) { m = d2; step_at = i; }
    }
    return m;
}
static uint32_t n_sw, last_sh, n_rs, last_dsw;
static void count_sw(void)                         /* (the rate changes seen; the restarts: fades begun) */
{
    if (stage() != last_sh) { n_sw++; last_sh = stage(); }
    if (fx.dsw == 1u && last_dsw != 1u) n_rs++;
    last_dsw = fx.dsw;
}
static uint32_t sweep_k;
static const uint8_t *sweep_seq;
static uint32_t sweep_n, sweep_every, sweep_t0;
static void sweep_hook(uint32_t t)
{
    count_sw();
    t -= sweep_t0;
    if (t / sweep_every != sweep_k && t / sweep_every < sweep_n) {
        sweep_k = t / sweep_every;
        song.g[G_DTIME] = sweep_seq[sweep_k];
    }
}
static int32_t bpm_seq[8];
static void bpm_hook(uint32_t t)
{
    count_sw();
    t -= sweep_t0;
    if (t / sweep_every != sweep_k && t / sweep_every < sweep_n) {
        sweep_k = t / sweep_every;
        song.g[G_BPM] = (int16_t)bpm_seq[sweep_k];
    }
}
static uint32_t jit_q;
static void jitter_hook(uint32_t t)
{
    count_sw();
    midi_beat_samples = jit_q + (uint32_t)((t / CTL) % 7u) * jit_q / 1000u - jit_q * 3u / 1000u;   /* +-0.3 % */
}
static void test_clicks(void)
{
    char what[220];
    int32_t own, st;
    uint32_t len;
    {   /* TIME through the free times, up past both boundaries and back, one step every 0.25 s, a sine playing */
        static uint8_t seq[64];
        uint32_t i, n = 0, last = DT_FREE + NELEM(DTIME_MS) - 1u;
        for (i = DT_FREE + 40u; i <= last; i++) seq[n++] = (uint8_t)i;
        for (i = last; i >= DT_FREE + 40u && n < NELEM(seq); i--) seq[n++] = (uint8_t)i;
        dreset();
        song.g[G_DFDBK] = 40;
        s_f = 220; s_a = 30000;
        song.g[G_DTIME] = seq[0];
        drun(sine_on, 0, 3u * FS);                 /* (the steady echo first: its own step) */
        own = step_max(2u * FS, 3u * FS);
        sweep_seq = seq; sweep_n = n; sweep_every = FS / 4u; sweep_k = 0; n_sw = n_rs = 0; last_sh = stage(); last_dsw = fx.dsw; sweep_t0 = g_now;
        len = n * sweep_every + FS;               /* (+1 s: the last change done) */
        if (len > NMAX - 3u * FS) len = NMAX - 3u * FS;
        blk_hook = sweep_hook;
        drun(sine_on, 3u * FS, len);
        blk_hook = 0;
        st = step_max(3u * FS, 3u * FS + len);
        snprintf(what, sizeof what, "TIME swept %s -> %s -> back, a sine playing: %u rate changes, largest step %d (the echo's own %d)",
                 N_DTIME[seq[0]], N_DTIME[last], n_sw, st, own);
        check(what, n_sw >= 4u && n_rs == n_sw && st <= own * 2 + 64);
    }
    {   /* the tempo: 1/2 at 120 -> 60 -> 120 -> 40 -> 200 BPM */
        static const int32_t B[6] = {120, 60, 120, 40, 200, 70};
        uint32_t i;
        dreset();
        song.g[G_DFDBK] = 40; song.g[G_DTIME] = 6;
        s_f = 220; s_a = 30000;
        drun(sine_on, 0, 2u * FS);
        own = step_max(FS, 2u * FS);
        for (i = 0; i < 6u; i++) bpm_seq[i] = B[i];
        sweep_n = 6; sweep_every = FS; sweep_k = 0; n_sw = n_rs = 0; last_sh = stage(); last_dsw = fx.dsw; sweep_t0 = g_now;
        blk_hook = bpm_hook;
        drun(sine_on, 2u * FS, 6u * FS);
        blk_hook = 0;
        st = step_max(2u * FS, 8u * FS);
        snprintf(what, sizeof what, "the tempo 120 -> 60 -> 120 -> 40 -> 200 -> 70 BPM at 1/2, a sine playing: %u rate changes, step %d (own %d, at %.3f s)",
                 n_sw, st, own, step_at / (double)FS);
        check(what, n_sw >= 3u && n_rs == n_sw && st <= own * 2 + 64);
    }
    {   /* a section / preset load: TIME 1/8 -> 4BAR at once, and back */
        static const uint8_t J[3] = {1, 9, 1};
        dreset();
        song.g[G_DFDBK] = 40; song.g[G_DTIME] = 1;
        s_f = 220; s_a = 30000;
        drun(sine_on, 0, 2u * FS);
        own = step_max(FS, 2u * FS);
        sweep_seq = J; sweep_n = 3; sweep_every = 2u * FS; sweep_k = 0; n_sw = n_rs = 0; last_sh = stage(); last_dsw = fx.dsw; sweep_t0 = g_now;
        blk_hook = sweep_hook;
        drun(sine_on, 2u * FS, 8u * FS);
        blk_hook = 0;
        st = step_max(2u * FS, 10u * FS);
        snprintf(what, sizeof what, "TIME 1/8 -> 4BAR -> 1/8 at once (a load), a sine playing: %u rate changes, step %d (own %d)",
                 n_sw, st, own);
        check(what, n_sw >= 2u && n_rs == n_sw && st <= own * 2 + 64);
    }
    {   /* an external clock wavering +-0.3 % about a boundary: one restart at most */
        uint32_t b;
        for (b = 0; b < 2u; b++) {
            dreset();
            jit_q = DLY_SPAN(b);
            at_samples(jit_q);
            s_f = 220; s_a = 20000;
            n_sw = 0; n_rs = 0; last_sh = stage(); last_dsw = fx.dsw;
            blk_hook = jitter_hook;
            drun(sine_on, 0, 6u * FS);
            blk_hook = 0;
            snprintf(what, sizeof what, "an external clock wavering +-0.3 %% at the %s boundary (%.3f s): %u restart(s) in 6 s",
                     b ? "half / quarter" : "full / half", jit_q / (double)FS, n_rs);
            check(what, n_rs == 1u && n_sw == 1u);
        }
    }
    {   /* after a change nothing of the old line: a burst at one rate, the change, then silence for good */
        static const uint32_t P[4][2] = {{0, 2}, {2, 0}, {1, 0}, {0, 1}};
        uint32_t p;
        for (p = 0; p < 4u; p++) {
            uint32_t a = t_of(P[p][0]), b = t_of(P[p][1]), sw = FS / 2u;
            int32_t after;
            dreset();
            song.g[G_DFDBK] = 100;
            at_samples(a);
            s_f = 440; s_a = 30000; s_len = FS / 4u;
            drun(sine_src, 0, sw);
            midi_beat_samples = b;
            drun(sine_src, sw, 64u * CTL);         /* (the fade, the clearing) */
            drun(sine_src, sw + 64u * CTL, b + a + FS / 2u);
            after = peak(L, sw + 16u * CTL, sw + 64u * CTL + b + a + FS / 2u);
            after |= peak(R, sw + 16u * CTL, sw + 64u * CTL + b + a + FS / 2u);
            snprintf(what, sizeof what, "rate %u -> %u: the old line never heard after the change (peak %d), the new rate %u",
                     P[p][0], P[p][1], after, stage());
            check(what, after == 0 && stage() == P[p][1]);
        }
    }
}

/* ------------------------------------------------------ 6. MOD, GRAIN, the rest --- */
static uint32_t crossings(const int32_t *x, uint32_t a, uint32_t b)
{
    uint32_t c = 0, i;
    for (i = a + 1u; i < b; i++) c += x[i - 1u] < 0 && x[i] >= 0;
    return c;
}
static void test_features(void)
{
    char what[220];
    uint32_t s;
    for (s = 1; s < 3u; s++) {
        uint32_t dl = t_of(s);
        int32_t p0, p1;
        double r0, r1;
        dreset(); at_samples(dl);
        s_f = 500; s_a = 30000; s_len = FS / 2u;
        drun(sine_src, 0, dl + FS / 2u + FS / 8u);
        r0 = rms(L, dl + FS / 8u, dl + FS / 2u); p0 = peak(L, 0, dl + FS);
        dreset(); at_samples(dl); song.g[G_DMOD] = 127;
        drun(sine_src, 0, dl + FS / 2u + FS / 8u);
        r1 = rms(L, dl + FS / 8u, dl + FS / 2u); p1 = peak(L, 0, dl + FS);
        snprintf(what, sizeof what, "rate %s: MOD 127 drifts the echo, its level %+.2f dB of MOD 0's (peaks %d / %d)",
                 s == 1u ? "half" : "1/4", 20 * log10(r1 / r0), p1, p0);
        check(what, fabs(20 * log10(r1 / r0)) < 1.0 && p1 < 40000);
    }
    for (s = 1; s < 3u; s++) {
        uint32_t dl = t_of(s), c0, c1;
        dreset(); at_samples(dl);
        s_f = 300; s_a = 30000; s_len = FS;
        drun(sine_src, 0, dl + FS);
        c0 = crossings(L, dl + FS / 4u, dl + 3u * FS / 4u);
        dreset(); at_samples(dl); song.g[G_DPIT] = 12;
        drun(sine_src, 0, dl + FS);
        c1 = crossings(L, dl + FS / 4u, dl + 3u * FS / 4u);
        snprintf(what, sizeof what, "rate %s: GRAIN PITCH +12 an octave up (zero crossings in 0.5 s: plain %u, +12 %u)",
                 s == 1u ? "half" : "1/4", c0, c1);
        check(what, c1 > c0 * 17u / 10u && c1 < c0 * 23u / 10u);
    }
    for (s = 1; s < 3u; s++) {
        uint32_t dl = t_of(s), k = (10u * FS) / dl;
        int32_t p1, pk;
        if (k < 3u) k = 3u;
        dreset(); at_samples(dl);
        song.g[G_DTYPE] = 1; song.g[G_DFDBK] = 100;
        s_f = 440; s_a = 30000; s_len = FS / 4u;
        drun(sine_src, 0, k * dl + FS / 2u);
        p1 = peak(L, dl, dl + FS / 2u);
        pk = peak(L, k * dl, k * dl + FS / 2u);
        snprintf(what, sizeof what, "rate %s: DIGITAL FEEDBK 100 holds the repeat (repeat %u %+.2f dB of the first)",
                 s == 1u ? "half" : "1/4", k, 20 * log10((double)pk / p1));
        check(what, fabs(20 * log10((double)pk / p1)) < 0.5);
    }
    {
        uint32_t dl = t_of(2), b;
        int32_t pk, pl;
        dreset(); at_samples(dl);
        song.g[G_DFDBK] = 120;
        s_f = 330; s_a = 60000; s_len = FS / 2u;
        drun(sine_src, 0, NMAX - FS);
        for (b = 1; (b + 1u) * dl + FS / 2u < NMAX - FS; b++) ;
        pk = peak(L, 0, NMAX - FS) > peak(R, 0, NMAX - FS) ? peak(L, 0, NMAX - FS) : peak(R, 0, NMAX - FS);
        pl = peak(L, b * dl, b * dl + FS / 2u);
        snprintf(what, sizeof what, "rate 1/4: ANALOG FEEDBK 120 grows and holds, bounded (peak %d; repeat %u %d, the first %d)",
                 pk, b, pl, peak(L, dl, dl + FS / 2u));
        check(what, pk < 70000 && pl >= peak(L, dl, dl + FS / 2u));
    }
    {
        uint32_t dl = t_of(0);                     /* PING-PONG: twice TIME past the full rate's end: half rate */
        int32_t l1, r1, l2, r2;
        dreset(); at_samples(dl);
        song.g[G_DPING] = 1; song.g[G_DFDBK] = 90;
        blk_hook = gate_open;
        drun(click_src, 0, 3u * dl + FS / 4u);
        blk_hook = 0;
        l1 = peak(L, dl - 300u, dl + 300u); r1 = peak(R, dl - 300u, dl + 300u);
        l2 = peak(L, 2u * dl - 300u, 2u * dl + 300u); r2 = peak(R, 2u * dl - 300u, 2u * dl + 300u);
        snprintf(what, sizeof what, "PING-PONG at %.2f s (half rate: %u): left %d / right %d, then left %d / right %d",
                 dl / (double)FS, stage(), l1, r1, l2, r2);
        check(what, stage() == 1u && l1 > 4 * r1 && r2 > 4 * l2);
    }
    {   /* TONE at half rate: the low-pass end darker (the late repeats of a noise burst), the low cut end thinner */
        uint32_t dl = t_of(1), tn;
        double e[3], lo[3];
        static const int16_t TN[3] = {64, 10, 118};
        for (tn = 0; tn < 3u; tn++) {
            uint32_t i;
            int32_t prev = 0;
            double hi = 0, all = 0, low = 0, lp = 0;
            dreset(); at_samples(dl);
            song.g[G_DFDBK] = 70; song.g[G_DCOLOR] = TN[tn];
            xs = 0x2468aceu;
            blk_hook = gate_open;
            drun(burst_src, 0, 4u * dl + FS / 4u);
            blk_hook = 0;
            for (i = 3u * dl; i < 3u * dl + FS / 4u; i++) {   /* the 3rd repeat: its high part (the first difference),
                                                         * its low part (a one-pole low-pass, ~150 Hz) */
                hi += (double)(L[i] - prev) * (L[i] - prev); prev = L[i];
                lp += (L[i] - lp) * 0.021;
                all += (double)L[i] * L[i];
                low += lp * lp;
            }
            e[tn] = hi / (all + 1); lo[tn] = low / (all + 1);
        }
        snprintf(what, sizeof what, "half rate TONE: the low-pass end darker (high share %.3f, open %.3f), the low cut end thinner (low share %.3f, open %.3f)",
                 e[1], e[0], lo[2], lo[0]);
        check(what, e[1] < e[0] / 4 && lo[2] < lo[0] / 4);
    }
    {   /* the gate: a single hit, a 2.5 s echo, nothing else sent: the echo comes back at its level */
        uint32_t dl = 5u * FS / 2u;
        int32_t p0, p1;
        dreset(); at_samples(FS / 2u);
        drun(click_src, 0, FS);
        p0 = peak(L, FS / 2u - 400u, FS / 2u + 400u);
        dreset(); at_samples(dl);
        drun(click_src, 0, dl + FS / 2u);
        p1 = peak(L, dl - 400u, dl + 400u);
        snprintf(what, sizeof what, "a 2.5 s echo after 2.5 s of nothing sent: back at its level (%d; at 0.5 s %d), the gate open",
                 p1, p0);
        check(what, p1 > p0 * 9 / 10);
    }
    {   /* (0.9.19) the gate between sparse repeats: a short TIME, FEEDBK 50, one click: every repeat after the
         * quiet's 1.5 s at its level (until now the gate shut in the gaps between them, each repeat came back
         * faded in over ~23 ms, -5.6 dB on its peak, and 0.9.14's rest cleared the line under the later ones) */
        uint32_t dl = FS / 2u, k;
        int32_t p[8];
        double r;
        int ok = 1;
        dreset(); at_samples(dl);
        song.g[G_DFDBK] = 50;
        memset(L, 0, sizeof L); memset(R, 0, sizeof R);
        drun(click_src, 0, 7u * dl + 800u);
        for (k = 1; k <= 7u; k++)
            p[k] = peak(L, k * dl - 400u, k * dl + 400u);
        r = (double)p[3] / p[2];                        /* the loop's own decay a repeat */
        for (k = 4; k <= 7u; k++)
            if ((double)p[k] < 0.75 * r * p[k - 1])
                ok = 0;
        snprintf(what, sizeof what, "sparse repeats at 0.5 s (FEEDBK 50): each past 1.5 s at its level (%d %d %d %d %d %d %d)",
                 p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        check(what, ok);
    }
}

/* ------------------------------------------------------------------ demos --- */
static void wav(const char *dir, const char *name, uint32_t n)
{
    char path[512];
    FILE *f;
    uint32_t i, v;
    snprintf(path, sizeof path, "%s/%s", dir, name);
    f = fopen(path, "wb");
    if (!f) return;
    fwrite("RIFF", 1, 4, f); v = 36u + n * 4u; fwrite(&v, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f); v = 16; fwrite(&v, 4, 1, f);
    { uint16_t h[2] = {1, 2}; fwrite(h, 2, 2, f); }
    v = FS; fwrite(&v, 4, 1, f); v = FS * 4u; fwrite(&v, 4, 1, f);
    { uint16_t h[2] = {4, 16}; fwrite(h, 2, 2, f); }
    fwrite("data", 1, 4, f); v = n * 4u; fwrite(&v, 4, 1, f);
    for (i = 0; i < n; i++) {
        int16_t s2[2] = {(int16_t)clamp(L[i] >> 1, -32767, 32767), (int16_t)clamp(R[i] >> 1, -32767, 32767)};
        fwrite(s2, 2, 2, f);
    }
    fclose(f);
    printf("delay: demo %s\n", path);
}
static int32_t pluck(uint32_t t)
{
    uint32_t k = t % (FS / 2u);
    return t < 2u * FS ? (int32_t)(40000.0 * exp(-(double)k / 3000.0) * sin(2 * M_PI * (t < FS ? 330 : 495) * k / FS)) : 0;
}
static void demos(const char *dir)
{
    dreset(); song.g[G_BPM] = 60; song.g[G_DTIME] = 6; song.g[G_DFDBK] = 60; song.g[G_DMOD] = 20; song.g[G_DMIX] = 90;
    drun(pluck, 0, 10u * FS);
    for (uint32_t i = 0; i < 10u * FS; i++) { L[i] += pluck(i) / 2; R[i] += pluck(i) / 2; }
    wav(dir, "delay_half_60bpm_1-2.wav", 10u * FS);
    dreset(); song.g[G_BPM] = 120; song.g[G_DTIME] = 8; song.g[G_DFDBK] = 60; song.g[G_DMOD] = 20; song.g[G_DMIX] = 90;
    drun(pluck, 0, 14u * FS);
    for (uint32_t i = 0; i < 14u * FS; i++) { L[i] += pluck(i) / 2; R[i] += pluck(i) / 2; }
    wav(dir, "delay_quarter_120bpm_2bar.wav", 14u * FS);
}
#endif

int main(int argc, char **argv)
{
    test_leak();
#ifdef DLY_SPAN
    test_times();
    test_timing();
    test_sound();
    test_clicks();
    test_features();
    if (argc > 1) demos(argv[1]);
#endif
    (void)argc; (void)argv;
    if (bad) {
        printf("delay test: %d FAILED\n", bad);
        return 1;
    }
    printf("delay test passed\n");
    return 0;
}
