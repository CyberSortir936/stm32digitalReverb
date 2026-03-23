#ifndef REVERB_DSP_H
#define REVERB_DSP_H

#include <stdint.h>

// --- Filter Structures ---

typedef struct {
    int16_t prev_out;
    int16_t damping; // 0..32767
} LPF_Filter;

// Comb or AllPass
typedef struct {
    int16_t *buffer;
    uint32_t size;
    uint32_t idx;
    int16_t feedback;
} Delay_Filter;

// --- Delay Line Structure ---

typedef struct {
    int16_t *buffer;
    uint32_t size;
    uint32_t idx;
} Delay_Line;

// --- Plate Reverb Structure ---

typedef struct {
    Delay_Filter diffuser[4]; //AllPass
    Delay_Filter tank_ap;   // AllPass
    Delay_Line main_delay;
    LPF_Filter damping_filter;

    int16_t decay;          // feedback
    int16_t mix;            // Wet/Dry

    int16_t loop_feedback_node; // Tank output
} Plate_Reverb;

// --- Function Prototypes ---

// Initialization
void LPF_Init(LPF_Filter *f, int16_t damping);
void Delay_Filter_Init(Delay_Filter *f, int16_t *buf, uint32_t size, int16_t feedback);
void Delay_Line_Init(Delay_Line *f, int16_t *buf, uint32_t size);

// Sample Processing
int16_t LPF_Process(LPF_Filter *f, int16_t input);
int16_t Comb_Process(Delay_Filter *f, int16_t input);
int16_t AllPass_Process(Delay_Filter *f, int16_t input);
int16_t Delay_Line_Process(Delay_Line *f, int16_t input);

// Plate Reverb Process
int16_t Plate_Process(Plate_Reverb *reverb, int16_t input);

#endif // REVERB_DSP_H