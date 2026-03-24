#include "reverb_dsp.h"

#define PS_BUF_SIZE 1024
static int16_t ps_buffer[PS_BUF_SIZE];
static uint32_t ps_write_ptr = 0;
static float ps_read_ptr = 0;

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
    
    // Явне приведення до int32_t перед множенням - КРИТИЧНО для STM32
    int32_t feedback_part = ((int32_t)out_sample * f->feedback) >> 15;
    int32_t new_val = (int32_t)input + feedback_part;

    // Жорсткий лімітер (Clipping)
    if (new_val > 32767) new_val = 32767;
    else if (new_val < -32768) new_val = -32768;

    f->buffer[f->idx] = (int16_t)new_val;
    f->idx++;
    if (f->idx >= f->size) f->idx = 0;

    return out_sample;
}

int16_t AllPass_Process(Delay_Filter *ap, int16_t input){
    int16_t bufout = ap->buffer[ap->idx];
    
    // out = bufout - (input * feedback)
    int32_t out32 = bufout - (((int32_t)input * ap->feedback) >> 15);
    // bufnew = input + (bufout * feedback)
    int32_t bufnew32 = input + (((int32_t)bufout * ap->feedback) >> 15);
    
    if(bufnew32 > 32767) bufnew32 = 32767;
    else if(bufnew32 < -32768) bufnew32 = -32768;
    
    ap->buffer[ap->idx] = (int16_t)bufnew32;
    
    ap->idx++;
    if (ap->idx >= ap->size) ap->idx = 0;
    
    if(out32 > 32767) out32 = 32767;
    else if(out32 < -32768) out32 = -32768;
    
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
    int16_t pd_out = Delay_Line_Process(&reverb->pre_delay, input);

    int32_t sum = 0;
    for(int i = 0; i < 8; i++) {
        sum += Comb_Process(&reverb->combs[i], pd_out); // Прибрали in_div, подаємо як є
    }
    
    // Сума 4-х фільтрів може бути великою, тому ділимо на 2 (>> 1)
    int16_t wet = (int16_t)(sum >> 1); 

    // Дифузія
    for(int i = 0; i < 4; i++) {
        wet = AllPass_Process(&reverb->allpasses[i], wet);
    }
    
    // Фільтрація тону
    return LPF_Process(&reverb->damping_filter, wet);
}

int16_t Plate_Process(Plate_Reverb *reverb, int16_t input, uint8_t shimmer_on) {
    int16_t temp = input;
    for(int i = 0; i < 4; i++) {
        temp = AllPass_Process(&reverb->diffuser[i], temp);
    }

    int32_t tank_in = (int32_t)temp + (((int32_t)reverb->loop_feedback_node * reverb->decay) >> 15);
    
    // ВИПРАВЛЕНО: всюди використовуємо tank_in
    if (tank_in > 32767) tank_in = 32767;
    else if (tank_in < -32768) tank_in = -32768;

    int16_t ap_out = AllPass_Process(&reverb->tank_ap, (int16_t)tank_in);
    
    if (shimmer_on) {
        int16_t shim = PitchShift_OctaveUp(ap_out);
        ap_out = (int16_t)(((int32_t)ap_out * 20000 >> 15) + ((int32_t)shim * 12000 >> 15));
    }

    int16_t delay_out = Delay_Line_Process(&reverb->main_delay, ap_out);
    reverb->loop_feedback_node = LPF_Process(&reverb->damping_filter, delay_out);

    return reverb->loop_feedback_node;
}

int16_t PitchShift_OctaveUp(int16_t input) {
    // Записуємо вхід у буфер
    ps_buffer[ps_write_ptr] = input;
    
    // Читаємо у 2 рази швидше (це дає октаву вгору)
    ps_read_ptr += 2.0f; 
    if (ps_read_ptr >= PS_BUF_SIZE) ps_read_ptr -= PS_BUF_SIZE;
    
    ps_write_ptr = (ps_write_ptr + 1) % PS_BUF_SIZE;

    // Проста лінійна інтерполяція, щоб не було тріску
    uint32_t i = (uint32_t)ps_read_ptr;
    uint32_t j = (i + 1) % PS_BUF_SIZE;
    float frac = ps_read_ptr - (float)i;
    
    int32_t out = (int32_t)((1.0f - frac) * ps_buffer[i] + frac * ps_buffer[j]);
    return (int16_t)out;
}