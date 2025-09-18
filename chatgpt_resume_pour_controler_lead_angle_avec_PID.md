========  ceci est un chatgpt résumé en markdown pour controler le lead angle avec un PID
========  il existe aussi un fichier avec le chat complet mais non formatté en markdown
Parfait ! 😄
Je vais te fournir le **Markdown complet en plusieurs blocs sûrs**, prêts à copier intégralement dans VSCode sans perte de contenu. Chaque bloc de code et section sera séparé pour éviter les "…".

---

# SPMSM Lead Angle et Contrôle – Markdown complet

## 1. Introduction

Ce document décrit le calcul et le contrôle du **lead angle** pour un moteur **SPMSM** avec contrôle sinusoidal, incluant :

* PID sur $i_d$
* LUT vitesse × courant (iq ≥ 0)
* Version float et Q15 (entiers)
* Génération 3 phases sinusoïdes PWM

---

## 2. Formule du lead angle SPMSM

$$
\Delta = \arctan\left(\frac{\omega_e L_s i_q}{R_s i_q + \omega_e \psi_f}\right)
$$

* $\omega_e$ : vitesse électrique \[rad/s]
* $L_s$ : inductance de phase \[H]
* $R_s$ : résistance de phase \[Ω]
* $\psi_f$ : flux rotor \[Wb]
* $i_q$ : courant de couple \[A]

---

## 3. Contrôle PID pour $i_d \approx 0$

$$
\theta_{\text{cmd}} = \theta_{\text{elec}} + \Delta_{\text{feedforward}} + \Delta_{\text{PID}}
$$

* $\Delta_{\text{feedforward}}$ : avance estimée (fonction de $\omega_e$ et $i_q$)
* $\Delta_{\text{PID}}$ : correction PID pour maintenir $i_d \approx 0$

Saturation : $\Delta_{\text{total}} \in [0°, 45°]$

---

## 4. Version float (C)

```c
#include <math.h>
#include <stdint.h>

#define RS 0.20f
#define LS 100e-6f
#define PSI_F 0.030f
#define KP 0.5f
#define KI 200.0f
#define DT 0.0001f
#define DELTA_MAX (30.0f*M_PI/180.0f)
#define IINT_MAX 20.0f
#define LPF_ALPHA 0.1f

static float Iint=0.0f;
static float id_filt=0.0f;

float compute_lead_angle(float id_meas, float iq_meas,
                         float theta_elec, float omega_e)
{
    id_filt = (1-LPF_ALPHA)*id_filt + LPF_ALPHA*id_meas;
    float err = -id_filt;

    Iint += KI*err*DT;
    if(Iint>IINT_MAX) Iint=IINT_MAX;
    if(Iint<-IINT_MAX) Iint=-IINT_MAX;

    float delta_pi = KP*err + Iint;
    float delta_ff = atan2f(-omega_e*LS*iq_meas, RS*iq_meas + omega_e*PSI_F);
    float delta = delta_pi + delta_ff;

    if(delta>DELTA_MAX) delta=DELTA_MAX;
    if(delta<-DELTA_MAX) delta=-DELTA_MAX;

    float theta_cmd = theta_elec + delta;
    while(theta_cmd>=2*M_PI) theta_cmd-=2*M_PI;
    while(theta_cmd<0) theta_cmd+=2*M_PI;

    return theta_cmd;
}
```

---

## 5. Version entiers / Q15 (C)

```c
#include <stdint.h>

#define Q15(x) ((int16_t)((x)*32767.0f))
#define MUL_Q15(a,b) ((int32_t)(a)*(int32_t)(b)>>15)
#define SAT_Q15(x) ((x)>32767?32767:((x)<-32768?-32768:(x)))

#define KP_Q15 Q15(0.05)
#define KI_Q15 Q15(0.001)
#define DELTA_MAX_Q15 Q15(0.25)

static int32_t Iint_q31=0;
static int16_t id_filt_q15=0;

int16_t compute_lead_angle_q15(int16_t id_meas_q15, int16_t theta_elec_q15)
{
    id_filt_q15 = id_filt_q15 - (id_filt_q15>>3) + (id_meas_q15>>3);
    int16_t err_q15 = -id_filt_q15;

    Iint_q31 += (int32_t)KI_Q15*err_q15;
    if(Iint_q31>(1<<30)) Iint_q31=(1<<30);
    if(Iint_q31<-(1<<30)) Iint_q31=-(1<<30);

    int16_t Iint_q15 = (int16_t)(Iint_q31>>15);
    int16_t delta_q15 = (int16_t)((((int32_t)KP_Q15*err_q15)>>15)+Iint_q15);

    if(delta_q15>DELTA_MAX_Q15) delta_q15=DELTA_MAX_Q15;
    if(delta_q15<-DELTA_MAX_Q15) delta_q15=-DELTA_MAX_Q15;

    int32_t theta_cmd_q15 = (int32_t)theta_elec_q15 + delta_q15;
    if(theta_cmd_q15>=32768) theta_cmd_q15-=32768;
    if(theta_cmd_q15<0) theta_cmd_q15+=32768;

    return (int16_t)theta_cmd_q15;
}
```

---

## 6. LUT vitesse × courant (iq ≥ 0)

```c
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define RS 0.20f
#define LS 100e-6f
#define PSI_F 0.030f
#define POLE_PAIRS 6
#define RPM_MAX 4000
#define IQ_MAX 20.0f
#define N_OMEGA 8
#define N_IQ 8

static int16_t rad_to_q15(float rad)
{
    if(rad<0) rad=0;
    if(rad>M_PI/4) rad=M_PI/4;
    return (int16_t)(rad/(2.0f*M_PI)*32768.0f);
}

int main(void)
{
    float omega_max = (RPM_MAX/60.0f)*2.0f*M_PI*POLE_PAIRS;
    printf("static const int16_t delta_table[%d][%d] = {\n",N_OMEGA,N_IQ);
    for(int i=0;i<N_OMEGA;i++){
        float omega = (float)i/(N_OMEGA-1)*omega_max;
        printf(" {");
        for(int j=0;j<N_IQ;j++){
            float iq = ((float)j/(N_IQ-1))*IQ_MAX;
            float delta_rad = atan2f(omega*LS*iq, RS*iq+omega*PSI_F);
            int16_t delta_q15 = rad_to_q15(delta_rad);
            printf("%5d", delta_q15);
            if(j<N_IQ-1) printf(", ");
        }
        printf(" }");
        if(i<N_OMEGA-1) printf(",");
        printf("\n");
    }
    printf("};\n");
    return 0;
}
```

---

## 7. Interpolation bilinéaire Q15

```c
#include <stdint.h>
#define N_OMEGA 8
#define N_IQ 8
extern const int16_t delta_table[N_OMEGA][N_IQ];
#define OMEGA_MAX_Q15 32767
#define IQ_MAX_Q15 32767

int16_t get_lead_angle_q15(int16_t omega_q15, int16_t iq_q15)
{
    if(omega_q15<0) omega_q15=0;
    if(omega_q15>OMEGA_MAX_Q15) omega_q15=OMEGA_MAX_Q15;
    if(iq_q15<0) iq_q15=0;
    if(iq_q15>IQ_MAX_Q15) iq_q15=IQ_MAX_Q15;

    int32_t u = (omega_q15*(N_OMEGA-1))>>15;
    int32_t v = (iq_q15*(N_IQ-1))>>15;
    int32_t du = (omega_q15*(N_OMEGA-1)) - (u<<15);
    int32_t dv = (iq_q15*(N_IQ-1)) - (v<<15);

    if(u>=N_OMEGA-1){u=N_OMEGA-2; du=32767;}
    if(v>=N_IQ-1){v=N_IQ-2; dv=32767;}

    int16_t d00=delta_table[u][v];
    int16_t d10=delta_table[u+1][v];
    int16_t d01=delta_table[u][v+1];
    int16_t d11=delta_table[u+1][v+1];

    int32_t d0=d00+(((int32_t)(d10-d00)*du)>>15);
    int32_t d1=d01+(((int32_t)(d11-d01)*du)>>15);
    int32_t d=d0+(((int32_t)(d1-d0)*dv)>>15);

    return (int16_t)d;
}
```

---

## 8. Génération 3 phases sinusoïdes PWM Q15

```c
#include <stdint.h>
#define SIN_LUT_SIZE 256
extern const int16_t sin_lut[SIN_LUT_SIZE];

void generate_3phases_q15(int16_t theta_q15,
                          int16_t *phaseA,
                          int16_t *phaseB,
                          int16_t *phaseC)
{
    int32_t index = ((int32_t)theta_q15*SIN_LUT_SIZE)>>15;
    int32_t idxA = index % SIN_LUT_SIZE;
    int32_t idxB = (idxA + SIN_LUT_SIZE/3) % SIN_LUT_SIZE;
    int32_t idxC = (idxA + 2*SIN_LUT_SIZE/3) % SIN_LUT_SIZE;

    *phaseA = sin_lut[idxA];
    *phaseB = sin_lut[idxB];
    *phaseC = sin_lut[idxC];
}

void control_loop_example(int16_t omega_q15, int16_t iq_q15, int16_t theta_elec_q15)
{
    int16_t delta_q15 = get_lead_angle_q15(omega_q15, iq_q15);
    int16_t theta_cmd = theta_elec_q15 + delta_q15;
    if(theta_cmd>=32768) theta_cmd-=32768;

    int16_t A,B,C;
    generate_3phases_q15(theta_cmd,&A,&B,&C);
    // A,B,C -> PWM outputs
}
```

---

## 9. Notes et recommandations

1. Lead angle total saturé dans $[0°, 45°]$.
2. LUT vitesse × courant évite tout float en boucle.
3. Interpolation bilinéaire Q15 améliore précision.
4. Sinusoïdes PWM générées à partir d’une LUT 256 points Q15.
5. LUT et LUT sinus générées offline et collées en `const` dans le firmware.

---

Si tu veux, je peux maintenant te fournir **un script Python pour générer automatiquement la LUT sinus Q15 256 points** et la LUT lead angle complète, prêt à copier dans ce Markdown.

Veux‑tu que je fasse ça ?



+++++++++++++++++++++++++++++++++++++ ci dessous toute la conversation avec chatgpt


