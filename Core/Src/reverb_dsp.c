#include "reverb_dsp.h"
#include <math.h>

#define PS_BUF_SIZE 4096 
static int16_t ps_buffer[PS_BUF_SIZE];
static uint32_t ps_write_ptr = 0;
static uint32_t ps_phase_int = 0;
static int32_t shimmer_lpf_state = 0;

// --- Filter Initializations ---

void Delay_Filter_Init(Delay_Filter *filter, int16_t *buf, uint32_t sz, int16_t fb) {
    filter->buffer = buf;
    filter->size = sz;
    filter->idx = 0;
    filter->feedback = fb;
    // Filling the buffer with zeroes to avoid noise
    for(uint32_t i = 0; i < sz; i++) filter->buffer[i] = 0; 
}

void LPF_Init(LPF_Filter *lpf, int16_t damping) {
    lpf->prev_out = 0;
    lpf->damping = damping;
}

void Delay_Line_Init(Delay_Line *f, int16_t *buf, uint32_t size){
    f->buffer = buf;
    f->size = size;
    f->idx = 0;
    // Filling the buffer with zeroes to avoid noise
    for(uint32_t i = 0; i < size; i++) f->buffer[i] = 0;
}

// --- Sample Processing ---

int16_t LPF_Process(LPF_Filter *lpf, int16_t input){
    int32_t diff = (int32_t)lpf->prev_out - input;
    int32_t accumulation = ((int32_t)lpf->damping * diff) >> 15;
    
    int32_t result = input + accumulation;

    // Clipping
    if (result > 32767) result = 32767;
    else if (result < -32768) result = -32768;

    lpf->prev_out = (int16_t)result;
    return lpf->prev_out;
}

int16_t Comb_Process(Delay_Filter *f, int16_t input) {
    int16_t out_sample = f->buffer[f->idx];
    
    int32_t feedback_part = ((int32_t)out_sample * f->feedback) >> 15;
    int32_t new_val = (int32_t)input + feedback_part;

    // Clipping
    if (new_val > 32767) new_val = 32767;
    else if (new_val < -32768) new_val = -32768;

    f->buffer[f->idx] = (int16_t)new_val;
    f->idx++;
    if (f->idx >= f->size) f->idx = 0;

    return out_sample;
}

int16_t AllPass_Process(Delay_Filter *ap, int16_t input){
    int16_t bufout = ap->buffer[ap->idx];
    
    // new node: w[n] = x[n] + g * w[n-D]
    int32_t bufnew32 = (int32_t)input + (((int32_t)bufout * ap->feedback) >> 15);
    
    // overflow protection
    if(bufnew32 > 32767) bufnew32 = 32767;
    else if(bufnew32 < -32768) bufnew32 = -32768;
    
    // output: y[n] = w[n-D] - g * w[n]
    int32_t out32 = (int32_t)bufout - (((int32_t)bufnew32 * ap->feedback) >> 15);
    
    // output limiter
    if(out32 > 32767) out32 = 32767;
    else if(out32 < -32768) out32 = -32768;
    
    // writing new sample in buffer
    ap->buffer[ap->idx] = (int16_t)bufnew32;
    
    // index shift
    ap->idx++;
    if (ap->idx >= ap->size) ap->idx = 0;
    
    return (int16_t)out32;
}

int16_t Delay_Line_Process(Delay_Line *f, int16_t input){
    int16_t output = f->buffer[f->idx];
    f->buffer[f->idx] = input;          
    
    f->idx++;
    if (f->idx >= f->size) f->idx = 0;
    
    return output;
}

// --- Reverb Algorithms ---
int16_t Hall_Process(Hall_Reverb *reverb, int16_t input, uint8_t shimmer_on) {
    // Pre Delay
    int16_t pd_out = Delay_Line_Process(&reverb->pre_delay, input);

    int32_t sum = 0;
    for(int i = 0; i < 8; i++) {
        sum += Comb_Process(&reverb->combs[i], pd_out);
    }
    
    // Dividing by 2 to avoid overflow
    int16_t wet = (int16_t)(sum >> 1); 

    // Shimmer is always active
    int16_t shim = PitchShift_OctaveUp(wet);
    shimmer_lpf_state = shimmer_lpf_state + (((shim - shimmer_lpf_state) * 4000) >> 15);

    if (shimmer_on) {
        wet = (int16_t)(((int32_t)wet * 16384 >> 15) + ((int32_t)shimmer_lpf_state * 24000 >> 15)); // Mix is determined by ear :)
    }

    // Diffusion
    for(int i = 0; i < 4; i++) {
        wet = AllPass_Process(&reverb->allpasses[i], wet);
    }
    
    // Tone Filter
    return LPF_Process(&reverb->damping_filter, wet);
}

int16_t Plate_Process(Plate_Reverb *reverb, int16_t input, uint8_t shimmer_on) {
    // Input diffusion
    int16_t temp = input;
    for(int i = 0; i < 4; i++) {
        temp = AllPass_Process(&reverb->diffuser[i], temp);
    }

    // Shimmer generation
    int16_t shim = 0;
    if (shimmer_on) {
        shim = PitchShift_OctaveUp(temp);
    } else {
        PitchShift_OctaveUp(0); 
    }

    // Tank input
    int32_t safe_decay = reverb->decay;
    if (safe_decay > 32600) safe_decay = 32600; 

    int32_t tank_in = (int32_t)temp;
    
    
    if (shimmer_on) {
        static int32_t plate_shim_lpf = 0;
        plate_shim_lpf = plate_shim_lpf + (((shim - plate_shim_lpf) * 5000) >> 15);
        tank_in += ((plate_shim_lpf * 20000) >> 15); 
    }
    
    tank_in += (((int32_t)reverb->loop_feedback_node * safe_decay) >> 15);
    
    // Soft-clipping
    if (tank_in > 24000) tank_in = 24000 + ((tank_in - 24000) >> 1);
    else if (tank_in < -24000) tank_in = -24000 + ((tank_in + 24000) >> 1); 

    if (tank_in > 32767) tank_in = 32767;
    else if (tank_in < -32768) tank_in = -32768;

    // In-Loop Processing
    int16_t ap_out = AllPass_Process(&reverb->tank_ap, (int16_t)tank_in);
    int16_t delay_out = Delay_Line_Process(&reverb->main_delay, ap_out);

    // DC Blocker
    static int32_t dc_lpf = 0;
    dc_lpf = dc_lpf + ((((int32_t)delay_out - dc_lpf) * 100) >> 15); // Фільтр ~10 Гц
    
    int32_t dc_blocked = (int32_t)delay_out - dc_lpf;
    
    if (dc_blocked > 32767) dc_blocked = 32767;
    else if (dc_blocked < -32768) dc_blocked = -32768;

    // Damp
    reverb->loop_feedback_node = LPF_Process(&reverb->damping_filter, (int16_t)dc_blocked);

    // Multitap
    uint32_t idx = reverb->main_delay.idx;
    uint32_t sz  = reverb->main_delay.size;

    // Three reading heads on different parts of the buffer
    int16_t tap1 = reverb->main_delay.buffer[(idx + sz - 3111) % sz];
    int16_t tap2 = reverb->main_delay.buffer[(idx + sz - 7333) % sz];
    int16_t tap3 = reverb->main_delay.buffer[(idx + sz - 11888) % sz];
    
    int32_t plate_out = ((int32_t)ap_out + tap1 + tap2 - tap3 + delay_out) * 16384 >> 15;

    if (plate_out > 32767) plate_out = 32767;
    else if (plate_out < -32768) plate_out = -32768;

    return (int16_t)plate_out;
}

int16_t PitchShift_OctaveUp(int16_t input) {
    ps_buffer[ps_write_ptr] = input;
    
    // Decreasing phase
    if (ps_phase_int == 0) {
        ps_phase_int = PS_BUF_SIZE - 1;
    } else {
        ps_phase_int--;
    }

    // Two phases reading two parts of buffer
    uint32_t ph1 = ps_phase_int;
    uint32_t ph2 = ph1 + (PS_BUF_SIZE / 2);
    if (ph2 >= PS_BUF_SIZE) ph2 -= PS_BUF_SIZE;


    int32_t idx1 = (int32_t)ps_write_ptr - (int32_t)ph1;
    if (idx1 < 0) idx1 += PS_BUF_SIZE;
    
    int32_t idx2 = (int32_t)ps_write_ptr - (int32_t)ph2;
    if (idx2 < 0) idx2 += PS_BUF_SIZE;

    int16_t tap1 = ps_buffer[idx1];
    int16_t tap2 = ps_buffer[idx2];

    // 5. Crossfade
    int32_t half = PS_BUF_SIZE / 2;
    int32_t v1;

    if (ph1 < half) {
        v1 = (ph1 * 32767) / half;
    } else {
        v1 = ((PS_BUF_SIZE - ph1) * 32767) / half;
    }
    
    int32_t v2 = 32767 - v1;

    // Blend two volumes
    int32_t out = ((int32_t)tap1 * v1 + (int32_t)tap2 * v2) >> 15;


    ps_write_ptr++;
    if (ps_write_ptr >= PS_BUF_SIZE) ps_write_ptr = 0;

    return (int16_t)out;
}