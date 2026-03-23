#include "reverb_dsp.h"

// --- Filter Initializations ---

void Delay_Filter_Init(Delay_Filter *filter, int16_t *buf, uint32_t sz, int16_t fb) {
    filter->buffer = buf;
    filter->size = sz;
    filter->idx = 0;
    filter->feedback = fb;
    // Обнуляємо буфер, щоб не було шуму при старті
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
    // Теж краще обнулити
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

int16_t Comb_Process(Delay_Filter *comb, int16_t input){
    int16_t out_sample = comb->buffer[comb->idx];
    
    // Feedback: input + (delay * feedback)
    int32_t new_val = input + ((int32_t)(out_sample * comb->feedback) >> 15);

    if (new_val > 32767) new_val = 32767;
    else if (new_val < -32768) new_val = -32768;

    comb->buffer[comb->idx] = (int16_t)new_val;
    
    comb->idx++;
    if (comb->idx >= comb->size) comb->idx = 0;

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